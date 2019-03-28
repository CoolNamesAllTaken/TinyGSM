/**
 * @file       TinyGsmClientSIM7000.h
 * @author     Volodymyr Shymanskyy
 * @license    LGPL-3.0
 * @copyright  Copyright (c) 2016 Volodymyr Shymanskyy
 * @date       Nov 2016
 */

#ifndef TinyGsmClientSIM7000_h
#define TinyGsmClientSIM7000_h

#define DEBUG_STREAM // print out skipped parts of stream

#define TINY_GSM_DEBUG Serial
//#define TINY_GSM_USE_HEX

#if !defined(TINY_GSM_RX_BUFFER)
  #define TINY_GSM_RX_BUFFER 64
#endif

#define TINY_GSM_MUX_COUNT 8

#include <TinyGsmCommon.h>

#define GSM_NL "\r\n"
static const char GSM_OK[] TINY_GSM_PROGMEM = "OK" GSM_NL;
static const char GSM_ERROR[] TINY_GSM_PROGMEM = "ERROR" GSM_NL;

static const uint32_t socket_update_interval_ms = 10000; // update interval for sockets, milliseconds

enum SimStatus {
  SIM_ERROR = 0,
  SIM_READY = 1,
  SIM_LOCKED = 2,
};

enum RegStatus {
  REG_UNREGISTERED = 0,
  REG_SEARCHING    = 2,
  REG_DENIED       = 3,
  REG_OK_HOME      = 1,
  REG_OK_ROAMING   = 5,
  REG_UNKNOWN      = 4,
};

enum TinyGSMDateTimeFormat {
  DATE_FULL = 0,
  DATE_TIME = 1,
  DATE_DATE = 2
};

class TinyGsmSim7000 : public TinyGsmModem
{

  public:
    class GsmClient : public Client
    {
      friend class TinyGsmSim7000;
      typedef TinyGsmFifo<uint8_t, TINY_GSM_RX_BUFFER> RxFifo;

      public:
        uint32_t       prev_check;
        bool           sock_connected;
        bool           got_data;

        GsmClient() {}

        GsmClient(TinyGsmSim7000& modem, uint8_t mux = 1) {
          init(&modem, mux);
        }

        bool init(TinyGsmSim7000* modem, uint8_t mux = 1) {
          this->at = modem;
          this->mux = mux;
          prev_check = 0;
          sock_connected = false;
          got_data = false;

          at->sockets[mux] = this;

          return true;
        }

        virtual int connect(const char *host, uint16_t port) {
          DBG("### CONNECTING TO UNSECURE GSM CLIENT");
          stop();
          TINY_GSM_YIELD();
          rx.clear();
          sock_connected = at->modemConnect(host, port, mux);
          return sock_connected;
        }

        virtual int connect(IPAddress ip, uint16_t port) {
          DBG("### CONNECTING TO UNSECURE GSM CLIENT OVER IP");
          String host; host.reserve(16);
          host += ip[0];
          host += ".";
          host += ip[1];
          host += ".";
          host += ip[2];
          host += ".";
          host += ip[3];
          return connect(host.c_str(), port);
        }

        virtual void stop() {
          TINY_GSM_YIELD();
          // Read and dump anything remaining in the modem's internal buffer.
          // The socket will appear open in response to connected() even after it
          // closes until all data is read from the buffer.
          // Doing it this way allows the external mcu to find and get all of the data
          // that it wants from the socket even if it was closed externally.
          rx.clear();
          // transfer from modem to FIFO and dump continuously until modem has no more data
          while (at->modemReadAll((uint16_t)rx.free(), mux)) {
            rx.clear();
          }
          at->sendAT(GF("+CACLOSE="), mux);
          sock_connected = false;
          at->waitResponse(5000L); // takes a while to close
        }

        /**
        * @brief Writes a given number of bytes from a buffer to the modem.
        *
        * @param buf    Pointer to the buffer to be writtne from.
        * @param size   Number of bytes to write to modem.
        *
        * @return Number of bytes successfully written to modem.
        **/
        virtual size_t write(const uint8_t *buf, size_t size) {
          TINY_GSM_YIELD();
          at->maintain();
          return at->modemSend(buf, size, mux);
        }

        /**
        * @brief  Writes a single byte to the modem.
        * 
        * @return 1 if byte is writte, 0 otherwise.
        **/
        virtual size_t write(uint8_t c) {
          return write(&c, 1);
        }

        /**
        * @brief  Writes a c string to the modem.
        *
        * @param str  C string to be written.
        *
        * @return The number of bytes (number of chars) written successfully.
        **/
        virtual size_t write(const char *str) {
          if (str == NULL) return 0;
          return write((const uint8_t *)str, strlen(str));
        }

        /**
        * @brief  Checks whether there is data in the rx FIFO waiting to be read.
        *
        * @return rx.size() How many bytes of data are waiting to be read from the FIFO.
        **/
        virtual int available() {
          TINY_GSM_YIELD();
          at->maintain();
          return rx.size();
        }

        /**
        * @brief  Transfer size # of bytes from rx FIFO to user buffer.  Keeps writing until
        *         size bytes have been transfered or modem is out of data.

        * @param  buf   Pointer to user buffer.
        * @param  size  Number of bytes to transfer into buffer.
        *
        * @return cnt   Number of bytes actually transferred into user buffer.
        **/
        virtual int read(uint8_t *buf, size_t size) {
          TINY_GSM_YIELD();
          size_t cnt = 0;
          while (cnt < size) {
            size_t chunk = TinyGsmMin(size-cnt, rx.size());
            if (chunk > 0) {
              rx.get(buf, chunk);
              buf += chunk;
              cnt += chunk;
              continue;
            }
            if (at->modemReadAll((uint16_t)rx.free(), mux) == 0) {
              break; // ran out of stuff to read from modem
            }
          }
          return cnt;
        }

        /**
        * @brief Returns a single byte from the rx FIFO.
        * 
        * @return c   Byte value (uint8_t), or -1 (int) if no byte was read.
        **/
        virtual int read() {
          uint8_t c;
          if (read(&c, 1) == 1) {
            return c;
          }
          return -1;
        }

        virtual int peek() { return -1; } //TODO
        virtual void flush() { at->stream.flush(); }

        virtual uint8_t connected() {
          DBG("### CHECKING CONNECTION");
          if (available()) {
            return true;
          }
          return sock_connected;
        }
        virtual operator bool() { return connected(); }

        /*
         * Extended API
         */

        String remoteIP() TINY_GSM_ATTR_NOT_IMPLEMENTED;

      private:
        TinyGsmSim7000* at;
        uint8_t        mux;
        // uint32_t       prev_check;
        // bool           sock_connected;
        // bool           got_data;
        RxFifo         rx;
    };


    class GsmClientSecure : public GsmClient
    {
      public:
        GsmClientSecure() {}

        GsmClientSecure(TinyGsmSim7000& modem, uint8_t mux = 1)
          : GsmClient(modem, mux)
        {}

        virtual int connect(const char *host, uint16_t port) {
          DBG("### CONNECTING TO SECURE GSM CLIENT");
          delay(5000);
          stop();
          TINY_GSM_YIELD();
          rx.clear();
          sock_connected = at->modemConnect(host, port, mux, true);
          return sock_connected;
        }
    };

    TinyGsmSim7000(Stream& stream)
      : TinyGsmModem(stream), stream(stream)
    {
      memset(sockets, 0, sizeof(sockets));
    }

    /*
     * Basic functions
     */

    bool init(const char* pin = NULL) {
      DBG(GF("### TinyGSM Version:"), TINYGSM_VERSION);
      if (!testAT()) {
        return false;
      }
      sendAT(GF("E0"));   // Echo Off
      if (waitResponse() != 1) {
        return false;
      }
      DBG(GF("### Modem:"), getModemName());
      getSimStatus();
      return true;
    }

    String getModemName() {
      return "SIMCom SIM7000";
    }

    void setBaud(unsigned long baud) {
      sendAT(GF("+IPR="), baud);
    }

    bool testAT(unsigned long timeout = 10000L) {
      for (unsigned long start = millis(); millis() - start < timeout; ) {
        // streamWrite(GF("AAAAA"));  // TODO: extra A's to help detect the baud rate
        sendAT(GF(""));
        if (waitResponse(200) == 1) return true;
        delay(100);
      }
      return false;
    }

    /**
    * @brief  Scroll through all active connections and read from them into the rx fifo.
    *         Set got_data flag to true for all fifos that were added to.
    **/
    void maintain() {
      // periodically check all connections when got_data flag is raised, populate sock_available
      // with whether there is data waiting to be read from the connection
      for (int mux = 0; mux < TINY_GSM_MUX_COUNT; mux++) {
        if (!sockets[mux]) {
          continue; // socket hasn't been populated, skip
        }
        GsmClient* sock = sockets[mux];
        if ((millis() - (sock->prev_check)) < socket_update_interval_ms) {
          continue; // socket has been checked recently, skip
        }
        // check socket
        sock->prev_check = millis();
        if (sock && sock->got_data) {
          if (modemReadAll((uint16_t)(sock->rx.free()), mux)) {
            // socket exists and it just dumped data into the FIFO
            sock->got_data = false;
          }
        }
      }
      while (stream.available()) {
        // clear out the stream with a timeout of 10 ms
        waitResponse(10, NULL, NULL);
      }
    }

    bool factoryDefault() {  // these commands aren't supported
      return false;
    }

    String getModemInfo() {
      sendAT(GF("I"));
      String res;
      if (waitResponse(1000L, res) != 1) {
        return "";
      }
      res.replace(GSM_NL "OK" GSM_NL, "");
      res.replace(GSM_NL, " ");
      res.trim();
      return res;
    }

    bool hasSSL() {
      return true;
    }

    bool hasWifi() {
      return false;
    }

    bool hasGPRS() {
      return true;
    }

    /*
     * Power functions
     */

    bool restart() {
      if (!testAT()) {
        return false;
      }
      //Enable Local Time Stamp for getting network time
      // TODO: Find a better place for this
      sendAT(GF("+CLTS=1"));
      if (waitResponse(10000L) != 1) {
        return false;
      }
      sendAT(GF("+CFUN=1,1"));
      if (waitResponse(10000L) != 1) {
        return false;
      }
      delay(3000);  //TODO:  Test this delay
      return init();
    }

    bool poweroff() {
      sendAT(GF("+CPOWD=1"));
      return waitResponse(GF("NORMAL POWER DOWN")) == 1;
    }

    bool radioOff() {
      sendAT(GF("+CFUN=0,0"));
      if (waitResponse(10000L) != 1) {
        return false;
      }
      delay(3000);
      return true;
    }

    /*
      During sleep, the SIM7000 module has its serial communication disabled.
      In order to reestablish communication pull the DRT-pin of the SIM7000 module
      LOW for at least 50ms. Then use this function to disable sleep mode.
      The DTR-pin can then be released again.
    */
    bool sleepEnable(bool enable = true) {
      sendAT(GF("+CSCLK="), enable);
      return waitResponse() == 1;
    }

    /*
     * SIM card functions
     */

    bool simUnlock(const char *pin) {
      sendAT(GF("+CPIN="), pin);
      return waitResponse() == 1;
    }

    String getSimCCID() {
      sendAT(GF("+ICCID"));
      if (waitResponse(GF(GSM_NL "+ICCID:")) != 1) {
        return "";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      res.trim();
      return res;
    }

    String getIMEI() {
      sendAT(GF("+GSN"));
      if (waitResponse(GF(GSM_NL)) != 1) {
        return "";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      res.trim();
      return res;
    }

    SimStatus getSimStatus(unsigned long timeout = 10000L) {
      for (unsigned long start = millis(); millis() - start < timeout; ) {
        sendAT(GF("+CPIN?"));
        if (waitResponse(GF(GSM_NL "+CPIN:")) != 1) {
          delay(1000);
          continue;
        }
        int status = waitResponse(GF("READY"), GF("SIM PIN"), GF("SIM PUK"));
        waitResponse();
        switch (status) {
          case 2:
          case 3:  return SIM_LOCKED;
          case 1:  return SIM_READY;
          default: return SIM_ERROR;
        }
      }
      return SIM_ERROR;
    }

    RegStatus getRegistrationStatus() {
      sendAT(GF("+CGREG?"));
      if (waitResponse(GF("+CGREG:")) != 1) {
        DBG("### Unhandled response when getting network registration status");
        return REG_UNKNOWN;
      }
      streamSkipUntil(','); // Skip format (0)
      int status = stream.readStringUntil('\n').toInt();
      DBG("### Network reg status: ", status);
      waitResponse(); // ignore newline with 'OK'
      return (RegStatus)status;
    }

    String getOperator() {
      sendAT(GF("+COPS?"));
      if (waitResponse(GF(GSM_NL "+COPS:")) != 1) {
        return "";
      }
      streamSkipUntil('"'); // Skip mode and format
      String res = stream.readStringUntil('"');
      waitResponse();
      return res;
    }

    /*
     * Generic network functions
     */

    int16_t getSignalQuality() {
      sendAT(GF("+CSQ"));
      if (waitResponse(GF(GSM_NL "+CSQ:")) != 1) {
        return 99;
      }
      int res = stream.readStringUntil(',').toInt();
      waitResponse();
      return res;
    }

    bool isNetworkConnected() {
      RegStatus s = getRegistrationStatus();
      return (s == REG_OK_HOME || s == REG_OK_ROAMING);
    }

    String getNetworkModes() {
      sendAT(GF("+CNMP=?"));
      if (waitResponse(GF(GSM_NL "+CNMP:")) != 1) {
        return "";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      return res;
    }

    String setNetworkMode(uint8_t mode) {
      sendAT(GF("+CNMP="), mode);
      if (waitResponse(GF(GSM_NL "+CNMP:")) != 1) {
        return "OK";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      return res;
    }

    String getPreferredModes() {
      sendAT(GF("+CMNB=?"));
      if (waitResponse(GF(GSM_NL "+CMNB:")) != 1) {
        return "";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      return res;
    }

    String setPreferredMode(uint8_t mode) {
      sendAT(GF("+CMNB="), mode);
      if (waitResponse(GF(GSM_NL "+CMNB:")) != 1) {
        return "OK";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      return res;
    }


    /*
     * GPRS functions
     */

    bool gprsConnect(const char* apn, const char* user = NULL, const char* pwd = NULL) {
      // gprsDisconnect();

      sendAT(GF("+CNACT=1,"), GF(apn)); // open wireless connection
      if (waitResponse() != 1) {
        DBG("### Unable to open wireless connection.");
        return false;
      }

      return true;
    }

    bool gprsDisconnect() {
      sendAT(GF("+CNACT=0")); // close wireless connection
      if (waitResponse() != 1) {
        DBG("### Unable to close wireless connection.");
        return false;
      }

      return true;
    }

    bool isGprsConnected() {
      sendAT(GF("+CNACT?"));
      if (waitResponse(GF(GSM_NL "+CNACT:")) != 1) {
        DBG("### Error while checking GPRS connection.");
        return false;
      }
      int status = stream.readStringUntil(',').toInt(); // 1 = active, 0 = inactive
      // skip ip address, 2 newlines and 'OK'
      waitResponse();
      if (status != 1)
        return false;

      return true;
    }

    /*
     * IP Address functions
     */

    String getLocalIP() {
      sendAT(GF("+CNACT?"));
      streamSkipUntil('\"');
      String ip_string = stream.readStringUntil('\"');
      return ip_string;
    }


    /*
     * Phone Call functions
     */

    bool setGsmBusy() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool callAnswer() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool callNumber() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool callHangup() TINY_GSM_ATTR_NOT_IMPLEMENTED;
    bool dtmfSend() TINY_GSM_ATTR_NOT_IMPLEMENTED;

    /*
     * Messaging functions
     */

    String sendUSSD(const String& code) {
      sendAT(GF("+CMGF=1"));
      waitResponse();
      sendAT(GF("+CSCS=\"HEX\""));
      waitResponse();
      sendAT(GF("+CUSD=1,\""), code, GF("\""));
      if (waitResponse() != 1) {
        return "";
      }
      if (waitResponse(10000L, GF(GSM_NL "+CUSD:")) != 1) {
        return "";
      }
      stream.readStringUntil('"');
      String hex = stream.readStringUntil('"');
      stream.readStringUntil(',');
      int dcs = stream.readStringUntil('\n').toInt();

      if (dcs == 15) {
        return TinyGsmDecodeHex8bit(hex);
      } else if (dcs == 72) {
        return TinyGsmDecodeHex16bit(hex);
      } else {
        return hex;
      }
    }

    bool sendSMS(const String& number, const String& text) {

      sendAT(GF("+AT+CSCA?"));
      waitResponse();
      sendAT(GF("+CMGF=1"));
      waitResponse();
      //Set GSM 7 bit default alphabet (3GPP TS 23.038)
      sendAT(GF("+CSCS=\"GSM\""));
      waitResponse();
      sendAT(GF("+CMGS=\""), number, GF("\""));
      if (waitResponse(GF(">")) != 1) { 
        return false;
      }
      stream.print(text);
      stream.write((char)0x1A);
      stream.flush();
      return waitResponse(60000L) == 1;
    }

    bool sendSMS_UTF16(const String& number, const void* text, size_t len) {
      sendAT(GF("+CMGF=1"));
      waitResponse();
      sendAT(GF("+CSCS=\"HEX\""));
      waitResponse();
      sendAT(GF("+CSMP=17,167,0,8"));
      waitResponse();

      sendAT(GF("+CMGS=\""), number, GF("\""));
      if (waitResponse(GF(">")) != 1) {
        return false;
      }

      uint16_t* t = (uint16_t*)text;
      for (size_t i=0; i<len; i++) {
        uint8_t c = t[i] >> 8;
        if (c < 0x10) { stream.print('0'); }
        stream.print(c, HEX);
        c = t[i] & 0xFF;
        if (c < 0x10) { stream.print('0'); }
        stream.print(c, HEX);
      }
      stream.write((char)0x1A);
      stream.flush();
      return waitResponse(60000L) == 1;
    }


    /*
     * Location functions
     */

    String getGsmLocation() {
      sendAT(GF("+CIPGSMLOC=1,1"));
      if (waitResponse(10000L, GF(GSM_NL "+CIPGSMLOC:")) != 1) {
        return "";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      res.trim();
      return res;
    }


    /*
     * GPS location functions
     */

    bool enableGPS() {
      sendAT(GF("+CGNSPWR=1"));
      if (waitResponse() != 1) {
        return false;
      }
      return true;
    }

    bool disableGPS() {
      sendAT(GF("+CGNSPWR=0"));
      if (waitResponse() != 1) {
        return false;
      }
      return true;
    }

    String getGPSraw() {
      sendAT(GF("+CGNSINF"));
      if (waitResponse(GF(GSM_NL "+CGNSINF:")) != 1) {
        return "";
      }
      String res = stream.readStringUntil('\n');
      waitResponse();
      res.trim();
      return res;
    }

    bool getGPS(float *lat, float *lon, float *speed=0, int *alt=0, int *vsat=0, int *usat=0) {
      //String buffer = "";
      bool fix = false;

      sendAT(GF("+CGNSINF"));
      if (waitResponse(GF(GSM_NL "+CGNSINF:")) != 1) {
        return false;
      }

      stream.readStringUntil(','); // mode
      if ( stream.readStringUntil(',').toInt() == 1 ) fix = true;
      stream.readStringUntil(','); //utctime
      *lat =  stream.readStringUntil(',').toFloat(); //lat
      *lon =  stream.readStringUntil(',').toFloat(); //lon
      if (alt != NULL) *alt =  stream.readStringUntil(',').toFloat(); //lon
      if (speed != NULL) *speed = stream.readStringUntil(',').toFloat(); //speed
      stream.readStringUntil(',');
      stream.readStringUntil(',');
      stream.readStringUntil(',');
      stream.readStringUntil(',');
      stream.readStringUntil(',');
      stream.readStringUntil(',');
      stream.readStringUntil(',');
      if (vsat != NULL) *vsat = stream.readStringUntil(',').toInt(); //viewed satelites
      if (usat != NULL) *usat = stream.readStringUntil(',').toInt(); //used satelites
      stream.readStringUntil('\n');

      waitResponse();

      return fix;
    }

    /*
     * Time functions
     */

    String getGSMDateTime(TinyGSMDateTimeFormat format) {
      sendAT(GF("+CCLK?"));
      if (waitResponse(2000L, GF(GSM_NL "+CCLK: \"")) != 1) {
        return "";
      }

      String res;

      switch(format) {
        case DATE_FULL:
          res = stream.readStringUntil('"');
        break;
        case DATE_TIME:
          streamSkipUntil(',');
          res = stream.readStringUntil('"');
        break;
        case DATE_DATE:
          res = stream.readStringUntil(',');
        break;
      }
      return res;
    }

    bool getGPSTime(int *year, int *month, int *day, int *hour, int *minute, int *second) {
      bool fix = false;
      char chr_buffer[12];
      sendAT(GF("+CGNSINF"));
      if (waitResponse(GF(GSM_NL "+CGNSINF:")) != 1) {
        return false;
      }

      for (int i = 0; i < 3; i++) {
        String buffer = stream.readStringUntil(',');
        buffer.toCharArray(chr_buffer, sizeof(chr_buffer));
        switch (i) {
          case 0:
            //mode
            break;
          case 1:
            //fixstatus
            if ( buffer.toInt() == 1 ) {
              fix = buffer.toInt();
            }
            break;
          case 2:
            *year = buffer.substring(0,4).toInt();
            *month = buffer.substring(4,6).toInt();
            *day = buffer.substring(6,8).toInt();
            *hour = buffer.substring(8,10).toInt();
            *minute = buffer.substring(10,12).toInt();
            *second = buffer.substring(12,14).toInt();
            break;

          default:
            // if nothing else matches, do the default
            // default is optional
            break;
        }
      }
      String res = stream.readStringUntil('\n');
      waitResponse();

      if (fix) {
        return true;
      } else {
        return false;
      }
    }

    /*
     * Battery functions
     */
    // Use: float vBatt = modem.getBattVoltage() / 1000.0;
    uint16_t getBattVoltage() {
      sendAT(GF("+CBC"));
      if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
        return 0;
      }
      streamSkipUntil(','); // Skip
      streamSkipUntil(','); // Skip

      uint16_t res = stream.readStringUntil(',').toInt();
      waitResponse();
      return res;
    }

    int8_t getBattPercent() {
      sendAT(GF("+CBC"));
      if (waitResponse(GF(GSM_NL "+CBC:")) != 1) {
        return false;
      }
      stream.readStringUntil(',');
      int res = stream.readStringUntil(',').toInt();
      waitResponse();
      return res;
    }

  /*
   * Client related functions
   */
  protected:
    GsmClient*    sockets[TINY_GSM_MUX_COUNT];

    bool modemConnect(const char* host, uint16_t port, uint8_t mux, bool ssl = false) {
      DBG("### modemConnect");
      if (ssl) {
        // use TLS
        sendAT(GF("+CACID="), mux); // configure the cid
        if (waitResponse() != 1) {
          DBG("### Error while setting <cid>.");
        }
        sendAT(GF("+CSSLCFG=\"sslversion\","), mux, GF(","), 3); // sslversion = TLS 1.2
        if (waitResponse() != 1) {
          DBG("### Error while configuring sslversion.");
        }
        sendAT(GF("+CASSLCFG="), mux, GF(",ssl,1")); // enable ssl for connection
        if (waitResponse() != 1) {
          DBG("### Error while enabling ssl for connection.");
        }
        // TODO: add certificate uploading and conversion (server only)
        // currently trusts all server certificates
        sendAT(GF("+CAOPEN="), mux, GF(",\""), GF(host), GF("\","), port); // connect to host
        if (waitResponse(5000L, GF("+CAOPEN:")) != 1) {
          DBG("### Error while connecting to host.");
        }
        streamSkipUntil(','); // skip cid
        uint8_t result = stream.readStringUntil('\n').toInt();
        if (result != 0) {
          DBG("### Error while connecting to host: result ", result, ".");
          return false;
        }
        // absorb two newlines and 'OK'
        waitResponse();
        return true;
      } else {
        // don't use TLS
        int rsp;
        sendAT(GF("+CIPSTART="), mux, ',', GF("\"TCP"), GF("\",\""), host, GF("\","), port);
        rsp = waitResponse(75000L,
                           GF("CONNECT OK" GSM_NL),
                           GF("CONNECT FAIL" GSM_NL),
                           GF("ALREADY CONNECT" GSM_NL),
                           GF("ERROR" GSM_NL),
                           GF("CLOSE OK" GSM_NL)   // Happens when HTTPS handshake fails
                          );
        return (1 == rsp);
      }
    }

    int16_t modemSend(const void* buff, size_t len, uint8_t mux) {
      sendAT(GF("+CASEND="), mux, GF(','), len);
      if (waitResponse(5000L, GF(">")) != 1) {
        DBG("### Didn't receive input prompt while trying to send data.");
        return 0;
      }
      stream.write((uint8_t*)buff, len);
      stream.flush();
      if (waitResponse() != 1) {
        DBG("### Error while sending data.");
        return 0;
      } else {
        streamSkipUntil(','); // Skip mux
        uint8_t result = stream.readStringUntil(',').toInt();
        uint16_t sendlen = stream.readStringUntil('\n').toInt();
        DBG("### Sent ", sendlen, " bytes of data with result ", result, ".");
        return sendlen;
      }
    }

    /**
    * @brief Reads at most max_len bytes from the modem by asking for data in 100-byte chunks.
    * 
    * @param max_len      Maximum number of bytes to read (ie. space left in rx buffer).
    * @param mux          Connection number (cid) of connection to read from.
    *
    * @return total_len   Number of bytes read.
    **/
    size_t modemReadAll(size_t max_len, uint8_t mux) {
      DBG("### modemReadAll");
      size_t len = 999;
      size_t total_len = 0;
      while (len > 0 && total_len < max_len) {
        sendAT(GF("+CARECV="), mux, GF(","), 100); // request 100 bytes from connection
        if (waitResponse(GF("+CARECV:")) != 1) {
          DBG("### modemReadAll Error");
          return 0;
        }
        streamSkipUntil(','); // skip cid
        len = stream.readStringUntil('\n').toInt(); // number of bytes actually returned

        // capture received data (len bytes)
        for (size_t i=0; i<len; i++) {
          while (!stream.available()) { TINY_GSM_YIELD(); }
          char c = stream.read();
          sockets[mux]->rx.put(c);
        }
        waitResponse(); // wait until 'OK'
        total_len += len;
      }
      DBG("### modemReadAll read ", total_len, " bytes");
      return total_len;
    }

    /**
    * @brief Check to see if modem is connected to the Cellular Network.
    *
    * @param mux  Connection number (cid) to check.
    *
    * @return     True if connection successful, false if not.
    **/
    bool modemGetConnected(uint8_t mux) {
      sendAT(GF("+CNACT?"));
      if (waitResponse(GF("1"), GF("0")) != 1) {
        return false;
      }
      streamSkipUntil('\"');
      String ip_string = stream.readStringUntil('\"');
      // consume two newlines and "OK"
      waitResponse();
      DBG("### IP Address:", ip_string.c_str());
      return true;
    }

  public:

    Stream&       stream;

    /*
     Utilities
     */

    template<typename... Args>
    void sendAT(Args... cmd) {
      streamWrite("AT", cmd..., GSM_NL);
      stream.flush();
      TINY_GSM_YIELD();
      DBG("AT", cmd...); // print out AT command being sent
    }

    // TODO: Optimize this!
    uint8_t waitResponse(uint32_t timeout, String& data,
                         GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                         GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
    {
      /*String r1s(r1); r1s.trim();
      String r2s(r2); r2s.trim();
      String r3s(r3); r3s.trim();
      String r4s(r4); r4s.trim();
      String r5s(r5); r5s.trim();
      DBG("### ..:", r1s, ",", r2s, ",", r3s, ",", r4s, ",", r5s);*/
      data.reserve(64);
      int index = 0;
      unsigned long startMillis = millis();
      do {
        TINY_GSM_YIELD();
        while (stream.available() > 0) {
          int a = stream.read();
          if (a <= 0) continue; // Skip 0x00 bytes, just in case
          data += (char)a;
          if (r1 && data.endsWith(r1)) {
            index = 1;
            goto finish;
          } else if (r2 && data.endsWith(r2)) {
            index = 2;
            goto finish;
          } else if (r3 && data.endsWith(r3)) {
            index = 3;
            goto finish;
          } else if (r4 && data.endsWith(r4)) {
            index = 4;
            goto finish;
          } else if (r5 && data.endsWith(r5)) {
            index = 5;
            goto finish;
          } /*else if (data.endsWith(GF(GSM_NL "+CARECV:"))) {
            // SIM7000 notifies of data reception
            int mux = stream.readStringUntil(',').toInt();
            int recvlen = stream.readStringUntil('\n').toInt();
            // TODO: read data here?  Does the modem even send CARECV?
            if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
              sockets[mux]->got_data = true;
            }
            data = "";
          } *//*else if (data.endsWith(GF("CLOSED" GSM_NL))) {
            // connection closed--only applies to CIP type commands
            int nl = data.lastIndexOf(GSM_NL, data.length()-8);
            int coma = data.indexOf(',', nl+2);
            int mux = data.substring(nl+2, coma).toInt();
            if (mux >= 0 && mux < TINY_GSM_MUX_COUNT && sockets[mux]) {
              sockets[mux]->sock_connected = false;
            }
            data = "";
            DBG("### Closed: ", mux);
          }*/
        }
      } while (millis() - startMillis < timeout);
      finish:
        #ifdef DEBUG_STREAM
        DBG(data);
        #endif
        if (!index) {
          data.trim();
          if (data.length()) {
            DBG("### Unhandled:", data);
          }
          data = "";
        }
        //DBG('<', index, '>');
        return index;
    }

    uint8_t waitResponse(uint32_t timeout,
                         GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                         GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
    {
      String data;
      return waitResponse(timeout, data, r1, r2, r3, r4, r5);
    }

    uint8_t waitResponse(GsmConstStr r1=GFP(GSM_OK), GsmConstStr r2=GFP(GSM_ERROR),
                         GsmConstStr r3=NULL, GsmConstStr r4=NULL, GsmConstStr r5=NULL)
    {
      return waitResponse(1000, r1, r2, r3, r4, r5);
    }
};

#endif
