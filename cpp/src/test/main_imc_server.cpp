#include <cstdlib>
#include <iostream>
#include <boost/bind.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>

#include "../../IMC/Base/Packet.hpp"
#include "../../IMC/Base/Message.hpp"

using boost::asio::ip::tcp;

const int max_length = 1024;

typedef boost::shared_ptr<tcp::socket> socket_ptr;

void session(socket_ptr sock)
{
    try
    {
        for (;;)
        {
//            char data[max_length];
            IMC::ByteBuffer bb = IMC::ByteBuffer();
            bb.setSize(max_length);

            boost::system::error_code error;
            size_t length = sock->read_some(boost::asio::buffer(bb.getBuffer(), bb.getSize()), error);
            if (error == boost::asio::error::eof)
                break; // Connection closed cleanly by peer.
            else if (error)
                throw boost::system::system_error(error); // Some other error.

            // Deserialize IMC packet
            auto pk = IMC::Packet();
            auto hb = pk.deserialize(bb.getBuffer(), bb.getSize());
            // Can I cast a Message to its message type? Heartbeat for instance

            std::cout << "Received: " << hb->toString() << std::endl;

            // Invert source and destination
            auto src = hb->getSource();
            auto src_ent = hb->getSourceEntity();
            auto dst = hb->getDestination();
            auto dst_ent = hb->getDestinationEntity();
            hb->setDestination(src);
            hb->setDestinationEntity(src_ent);
            hb->setSource(dst);
            hb->setSourceEntity(dst_ent);

            // Serialize the message
            pk.serialize(hb, bb);

            std::cout << "Sent: " << hb->toString() << std::endl;

            // Send
            boost::asio::write(*sock, boost::asio::buffer(bb.getBuffer(), bb.getSize()));
        }
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception in thread: " << e.what() << "\n";
    }
}

void server(boost::asio::io_service& io_service, short port)
{
    tcp::acceptor a(io_service, tcp::endpoint(tcp::v4(), port));
    for (;;)
    {
        socket_ptr sock(new tcp::socket(io_service));
        a.accept(*sock);
        boost::thread t(boost::bind(session, sock));
    }
}

int main(int argc, char* argv[])
{
    try
    {
        if (argc != 2)
        {
            std::cerr << "Usage: blocking_tcp_echo_server <port>\n";
            return 1;
        }

        boost::asio::io_service io_service;

        using namespace std; // For atoi.
        server(io_service, atoi(argv[1]));
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << "\n";
    }

    return 0;
}