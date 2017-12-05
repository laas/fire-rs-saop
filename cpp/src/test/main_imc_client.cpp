#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#include "../../IMC/Spec/Heartbeat.hpp"
#include "../../IMC/Base/Packet.hpp"

using boost::asio::ip::tcp;

enum { max_length = 1024 };

int main(int argc, char* argv[])
{
    try
    {
        if (argc != 3)
        {
            std::cerr << "Usage: blocking_tcp_echo_client <host> <port>\n";
            return 1;
        }

        boost::asio::io_service io_service;

        // Connect to the server
        tcp::socket s(io_service);
        tcp::resolver resolver(io_service);
        boost::asio::connect(s, resolver.resolve({argv[1], argv[2]}));

        std::cout << "Enter message: ";
        char request[max_length];
        std::cin.getline(request, max_length);
//        size_t request_length = std::strlen(request);

        auto hb = std::unique_ptr<IMC::Heartbeat>(new IMC::Heartbeat());
        hb.get()->setTimeStamp(25632);
        auto pk = IMC::Packet();
        IMC::ByteBuffer bb = IMC::ByteBuffer();
        pk.serialize(static_cast<IMC::Message*>(hb.get()), bb);


        // Send message
        boost::asio::write(s, boost::asio::buffer(bb.getBuffer(), bb.getSize()));

//        // Get reply
//        char reply[max_length];
//        size_t reply_length = boost::asio::read(s,
//                                                boost::asio::buffer(reply, bb.getSize()));
//        std::cout << "Reply is: ";
//        std::cout.write(reply, reply_length);
//        std::cout << "\n";
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}

