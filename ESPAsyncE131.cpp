/*
* ESPAsyncE131.cpp
*
* Project: ESPAsyncE131 - Asynchronous E.131 (sACN) library for Arduino ESP8266 and ESP32
* Copyright (c) 2019 Shelby Merrick
* http://www.forkineye.com
*
*  This program is provided free for you to use in any way that you wish,
*  subject to the laws and regulations where you are using it.  Due diligence
*  is strongly suggested before using this code.  Please give credit where due.
*
*  The Author makes no warranty of any kind, express or implied, with regard
*  to this program or the documentation contained in this document.  The
*  Author shall not be liable in any event for incidental or consequential
*  damages in connection with, or arising out of, the furnishing, performance
*  or use of these programs.
*
*/

#include "ESPAsyncE131.h"

// E1.17 ACN Packet Identifier
const byte ESPAsyncE131::ACN_ID[12] = { 0x41, 0x53, 0x43, 0x2d, 0x45, 0x31, 0x2e, 0x31, 0x37, 0x00, 0x00, 0x00 };

// Constructor
ESPAsyncE131::ESPAsyncE131(uint8_t buffers) {
    stats.num_packets = 0;
    stats.packet_errors = 0;
}

/////////////////////////////////////////////////////////
//
// Public begin() members
//
/////////////////////////////////////////////////////////

bool ESPAsyncE131::begin(e131_listen_t type, uint16_t universe, uint8_t n, protocol_t protocol) {
    return begin(type, E131_ListenPort, universe, n, protocol);
}


bool ESPAsyncE131::begin(e131_listen_t type, ESPAsyncE131PortId UdpPortId, uint16_t universe, uint8_t n, protocol_t protocol) {
    bool success = false;

    currentProtocol = protocol;
    E131_ListenPort = (protocol == PROTOCOL_ARTNET) ? ARTNET_DEFAULT_PORT : UdpPortId;

    if (protocol == PROTOCOL_E131) {
        if (type == E131_UNICAST)
            success = initUnicast();
        if (type == E131_MULTICAST)
            success = initMulticast(universe, n);
    } else if (protocol == PROTOCOL_ARTNET) {
        if (type == E131_UNICAST)
            success = initArtnetUnicast();
        if (type == E131_MULTICAST)
            success = initArtnetBroadcast();
    }

    if (success) {
        log_i("Initialized %s protocol on port %d", 
              protocol == PROTOCOL_E131 ? "E1.31" : "Art-Net", 
              E131_ListenPort);
    } else {
        log_e("Failed to initialize %s protocol", 
              protocol == PROTOCOL_E131 ? "E1.31" : "Art-Net");
    }

    return success;
}

/////////////////////////////////////////////////////////
//
// Private init() members
//
/////////////////////////////////////////////////////////

bool ESPAsyncE131::initUnicast() {
    bool success = false;
    delay(100);

    if (udp.listen(E131_ListenPort)) {
        udp.onPacket(std::bind(&ESPAsyncE131::parsePacket, this,
                std::placeholders::_1));
        success = true;
    }
    return success;
}

bool ESPAsyncE131::initMulticast(uint16_t universe, uint8_t n) {
    bool success = false;
    delay(100);

    IPAddress address = IPAddress(239, 255, ((universe >> 8) & 0xff),
        ((universe >> 0) & 0xff));

    if (udp.listenMulticast(address, E131_ListenPort)) {
        ip4_addr_t ifaddr;
        ip4_addr_t multicast_addr;

        ifaddr.addr = static_cast<uint32_t>(WiFi.localIP());
        for (uint8_t i = 1; i < n; i++) {
            multicast_addr.addr = static_cast<uint32_t>(IPAddress(239, 255,
                    (((universe + i) >> 8) & 0xff), (((universe + i) >> 0)
                    & 0xff)));
            igmp_joingroup(&ifaddr, &multicast_addr);
        }

        udp.onPacket(std::bind(&ESPAsyncE131::parsePacket, this,
                std::placeholders::_1));

        success = true;
    }
    return success;
}

bool ESPAsyncE131::initArtnetUnicast() {
    bool success = false;
    delay(100);

    log_i("Initializing Art-Net unicast on port %d", ARTNET_DEFAULT_PORT);
    if (udp.listen(ARTNET_DEFAULT_PORT)) {
        udp.onPacket(std::bind(&ESPAsyncE131::parsePacket, this, std::placeholders::_1));
        success = true;
        log_i("Art-Net unicast initialized successfully");
    } else {
        log_e("Failed to initialize Art-Net unicast");
    }
    return success;
}

bool ESPAsyncE131::initArtnetBroadcast() {
    bool success = false;
    delay(100);

    log_i("Initializing Art-Net broadcast on port %d", ARTNET_DEFAULT_PORT);
    if (udp.listen(ARTNET_DEFAULT_PORT)) {
        udp.onPacket(std::bind(&ESPAsyncE131::parsePacket, this, std::placeholders::_1));
        success = true;
        log_i("Art-Net broadcast initialized successfully");
    } else {
        log_e("Failed to initialize Art-Net broadcast");
    }
    return success;
}

/////////////////////////////////////////////////////////
//
// Packet parsing - Private
//
/////////////////////////////////////////////////////////

void ESPAsyncE131::parsePacket(AsyncUDPPacket _packet) {
    if (memcmp(_packet.data(), ARTNET_ID, sizeof(ARTNET_ID)) == 0) {
        parseArtnetPacket(_packet);
        return;
    }

    e131_error_t error = ERROR_NONE;

    sbuff = reinterpret_cast<e131_packet_t *>(_packet.data());
    if (memcmp(sbuff->acn_id, ESPAsyncE131::ACN_ID, sizeof(sbuff->acn_id)))
        error = ERROR_ACN_ID;
    if (htonl(sbuff->root_vector) != ESPAsyncE131::VECTOR_ROOT)
        error = ERROR_VECTOR_ROOT;
    if (htonl(sbuff->frame_vector) != ESPAsyncE131::VECTOR_FRAME)
        error = ERROR_VECTOR_FRAME;
    if (sbuff->dmp_vector != ESPAsyncE131::VECTOR_DMP)
        error = ERROR_VECTOR_DMP;
    if (sbuff->property_values[0] != 0)
        error = ERROR_IGNORE;

    if (!error) {
        dataReceived = true;
        if (PacketCallback) { 
            PacketCallback(sbuff, PROTOCOL_E131, UserInfo); 
        }

        stats.num_packets++;
        stats.last_clientIP = _packet.remoteIP();
        stats.last_clientPort = _packet.remotePort();
        stats.last_seen = millis();
    } else if (error == ERROR_IGNORE) {
    } else {
        dumpError(error);
        stats.packet_errors++;
    }
}

void ESPAsyncE131::parseArtnetPacket(AsyncUDPPacket _packet) {
    artnet_dmx_packet_t* artnetPacket = reinterpret_cast<artnet_dmx_packet_t*>(_packet.data());
    
    // Verify packet ID
    if (memcmp(artnetPacket->id, ARTNET_ID, sizeof(ARTNET_ID)) != 0) {
        log_w("Invalid Art-Net ID");
        return;
    }

    // Check the opcode to determine the type of Art-Net packet
    uint16_t opcode = (_packet.data()[9] << 8) | _packet.data()[8];

    switch (opcode) {
        case 0x5000:  // ArtDMX
            dataReceived = true;
            if (PacketCallback) { 
                PacketCallback(_packet.data(), PROTOCOL_ARTNET, UserInfo); 
            }
            
            stats.num_packets++;
            stats.last_clientIP = _packet.remoteIP();
            stats.last_clientPort = _packet.remotePort();
            stats.last_seen = millis();
            break;

        case 0x2000:  // ArtPoll
            log_i("Received ArtPoll packet");
            break;

        default:
            log_w("Unknown Art-Net opcode: 0x%04X", opcode);
            break;
    }
}


/////////////////////////////////////////////////////////
//
// Debugging functions - Public
//
/////////////////////////////////////////////////////////


void ESPAsyncE131::dumpError(e131_error_t error) {
    switch (error) {
        case ERROR_ACN_ID:
            log_e("INVALID PACKET ID");
            break;
        case ERROR_PACKET_SIZE:
            log_e("INVALID PACKET SIZE");
            break;
        case ERROR_VECTOR_ROOT:
            log_e("INVALID ROOT VECTOR: 0x%08X", htonl(sbuff->root_vector));
            break;
        case ERROR_VECTOR_FRAME:
            log_e("INVALID FRAME VECTOR: 0x%08X", htonl(sbuff->frame_vector));
            break;
        case ERROR_VECTOR_DMP:
            log_e("INVALID DMP VECTOR: 0x%02X", sbuff->dmp_vector);
            break;
        case ERROR_NONE:
        case ERROR_IGNORE:
            break;
    }
}
