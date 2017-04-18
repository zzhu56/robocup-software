#include "SimRadio.hpp"

#include <Network.hpp>
#include <stdexcept>
#include <iostream>
#include "firmware-common/common2015/utils/rtp.hpp"
#include "google/protobuf/io/zero_copy_stream_impl_lite.h"
#include "google/protobuf/io/gzip_stream.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "protobuf/nanopb/RadioTx.pb.h"
#include <array>
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
    google::protobuf::io::StringOutputStream stream(&out);

    auto options = google::protobuf::io::GzipOutputStream::Options();
    options.format = google::protobuf::io::GzipOutputStream::ZLIB;
    // options.compression_level = 9;
    google::protobuf::io::GzipOutputStream gzip(&stream, options);
    packet.SerializeToZeroCopyStream(&gzip);

    

    cout<<"test"<<gzip.ByteCount()<<endl;

    gzip.Close();
    cout<<"test1:"<<out.size()<<endl;



    packet.SerializeToString(&out);

    /*
    Packet_RobotsTxPacket p = Packet_RobotsTxPacket_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(reinterpret_cast<const unsigned char *>( &out[0]), out.length()+1);
    auto status = pb_decode(&stream, Packet_RobotsTxPacket_fields, &p);
    cout<<status<<endl;

    std::string buffer;
    buffer.resize(Packet_RobotsTxPacket_size);
    auto oStream = pb_ostream_from_buffer(reinterpret_cast<unsigned char *>(&buffer[0]), buffer.size());
    status = pb_encode(&oStream, Packet_RobotsTxPacket_fields, &p);
    cout<<status<<endl;
    cout<<oStream.bytes_written<<endl;

    Packet::RobotsTxPacket asdf;
    asdf.ParseFromString(buffer);
    cout<<(asdf.SerializeAsString() == packet.SerializeAsString())<<endl;

     */
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
