/*
  ESPAsyncDNSServer.h - Minimal Async DNS Server for ESP32
  Compatible with ESPAsync_WiFiManager (khoih-prog)
  
  This is a minimal implementation that provides the AsyncDNSServer class
  used by ESPAsync_WiFiManager for the captive portal.
*/

#ifndef ESPAsyncDNSServer_h
#define ESPAsyncDNSServer_h

#include <Arduino.h>
#include <WiFi.h>
#include <AsyncTCP.h>

// DNS reply codes
enum class AsyncDNSReplyCode : uint8_t {
  NoError = 0,
  FormError = 1,
  ServerFailure = 2,
  NonExistentDomain = 3,
  NotImplemented = 4,
  Refused = 5
};

class AsyncDNSServer {
public:
  AsyncDNSServer() : _port(53), _running(false) {}

  // Start the DNS server on the given port, responding to all queries
  // with the given IP address
  bool start(uint16_t port, const char* domain, IPAddress ip) {
    _port = port;
    _ip = ip;
    _domain = domain;
    
    if (_running) {
      stop();
    }
    
    _server = new AsyncServer(port);
    if (!_server) {
      return false;
    }
    
    _server->onClient([this](void* arg, AsyncClient* client) {
      if (client) {
        client->onData([this](void* arg, AsyncClient* c, void* data, size_t len) {
          handleRequest(c, (uint8_t*)data, len);
        }, NULL);
        client->onDisconnect([](void* arg, AsyncClient* c) {
          if (c) c->free();
        }, NULL);
      }
    }, NULL);
    
    _server->begin();
    _running = true;
    return true;
  }

  void stop() {
    if (_server) {
      _server->end();
      delete _server;
      _server = nullptr;
    }
    _running = false;
  }

  void setErrorReplyCode(AsyncDNSReplyCode code) {
    _errorCode = code;
  }

  void setTTL(uint32_t ttl) {
    _ttl = ttl;
  }

private:
  void handleRequest(AsyncClient* client, uint8_t* data, size_t len) {
    if (len < 12) return; // DNS header is at least 12 bytes
    
    // Parse DNS header
    uint16_t id = (data[0] << 8) | data[1];
    uint16_t flags = (data[2] << 8) | data[3];
    uint16_t qdcount = (data[4] << 8) | data[5];
    
    // Build response
    uint8_t response[512];
    memset(response, 0, sizeof(response));
    
    // Copy header
    memcpy(response, data, 12);
    
    // Set response flags (QR=1, opcode, RD, RA)
    response[2] = 0x80 | (flags & 0x78); // QR=1, keep opcode
    response[3] = 0x80; // RA=1
    
    // Set answer count to 1
    response[6] = 0;
    response[7] = 1;
    
    // Copy question section
    size_t offset = 12;
    size_t qlen = 0;
    // Find end of question (QNAME + QTYPE + QCLASS)
    while (offset < len && data[offset] != 0) {
      offset += data[offset] + 1;
    }
    if (offset >= len) return;
    offset += 1; // skip null byte
    offset += 4; // skip QTYPE + QCLASS
    qlen = offset;
    
    if (qlen > sizeof(response) - 12) return;
    memcpy(response + 12, data + 12, qlen - 12);
    
    // Add answer section
    size_t ansOffset = qlen;
    
    // Pointer to question name (0xC00C)
    response[ansOffset++] = 0xC0;
    response[ansOffset++] = 0x0C;
    
    // Type A (1)
    response[ansOffset++] = 0x00;
    response[ansOffset++] = 0x01;
    
    // Class IN (1)
    response[ansOffset++] = 0x00;
    response[ansOffset++] = 0x01;
    
    // TTL
    uint32_t ttl = _ttl;
    response[ansOffset++] = (ttl >> 24) & 0xFF;
    response[ansOffset++] = (ttl >> 16) & 0xFF;
    response[ansOffset++] = (ttl >> 8) & 0xFF;
    response[ansOffset++] = ttl & 0xFF;
    
    // RDLENGTH = 4 (IPv4)
    response[ansOffset++] = 0x00;
    response[ansOffset++] = 0x04;
    
    // RDATA (IP address)
    response[ansOffset++] = _ip[0];
    response[ansOffset++] = _ip[1];
    response[ansOffset++] = _ip[2];
    response[ansOffset++] = _ip[3];
    
    if (client) {
      client->write((const char*)response, ansOffset);
    }
  }

  uint16_t _port;
  bool _running;
  IPAddress _ip;
  String _domain;
  AsyncDNSReplyCode _errorCode = AsyncDNSReplyCode::NoError;
  uint32_t _ttl = 60;
  AsyncServer* _server = nullptr;
};

#endif
