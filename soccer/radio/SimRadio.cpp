#include "SimRadio.hpp"

#include <Network.hpp>
#include <stdexcept>
#include <iostream>
#include "firmware-common/common2015/utils/rtp.hpp"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/io/gzip_stream.h"
using namespace std;
using namespace Packet;

static QHostAddress LocalAddress(QHostAddress::LocalHost);

SimRadio::SimRadio(bool blueTeam) {
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
}

bool SimRadio::isOpen() const {
    // FIXME - check the socket
    return true;
}

void SimRadio::send(Packet::RobotsTxPacket& packet) {
    std::string out;
    //google::protobuf::io::StringOutputStream stream(&out);
    //google::protobuf::io::GzipOutputStream gzip(&stream);
    packet.SerializeToString(&out);
    //cout<<gzip.ByteCount()<<endl;
    //gzip.Close();
    //cout<<stream.ByteCount()<<endl;
    //cout<<"send packet"<<rtp::Forward_Size<<endl;
    _socket.writeDatagram(&out[0], out.size(), LocalAddress,
                          RadioTxPort + _channel);
}

void SimRadio::receive() {
    while (_socket.hasPendingDatagrams()) {
        unsigned int n = _socket.pendingDatagramSize();
        string buf;
        buf.resize(n);
        _socket.readDatagram(&buf[0], n);

        _reversePackets.push_back(RobotRxPacket());
        auto& packet = _reversePackets.back();

        if (!packet.ParseFromString(buf)) {
            printf("Bad radio packet of %d bytes\n", n);
            continue;
        }
    }
}

void SimRadio::switchTeam(bool blueTeam) {
    _socket.close();
    _channel = blueTeam ? 1 : 0;
    if (!_socket.bind(RadioRxPort + _channel)) {
        throw runtime_error(QString("Can't bind to the %1 team's radio port.")
                                .arg(blueTeam ? "blue" : "yellow")
                                .toStdString());
    }
}
