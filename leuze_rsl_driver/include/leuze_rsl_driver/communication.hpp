// Copyright 2019 Fraunhofer Institute for Manufacturing Engineering and Automation (IPA)
// Copyright 2019 Leuze electronic GmbH + Co. KG
// Licensed under the Apache License, Version 2.0

#ifndef LEUZE_RSL_DRIVER__COMMUNICATION_HPP_
#define LEUZE_RSL_DRIVER__COMMUNICATION_HPP_

#include <condition_variable>
#include <iostream>
#include <string>
#include <mutex>
#include <boost/asio.hpp>
#include <boost/bind/bind.hpp>
#include <boost/thread.hpp>
#include "udp_sim.hpp"

class DataParser
{
public:
  virtual int parseBuffer(std::basic_string<unsigned char> buffer) = 0;
};

class Connection
{
public:
  void start_read(std::size_t n)
  {
    async_read(n, &Connection::handle_packet);
    io_service_thread = boost::thread(boost::bind(&boost::asio::io_service::run, &io_service));
  }

  bool is_connected()
  {
    return is_connected_;
  }

  bool checkConnection()
  {
    if (!is_connected())
      return false;
    if ((std::time(0) - last_data_time) > 2)
    {
      disconnect();
      return false;
    }
    return true;
  }

  void disconnect()
  {
    is_connected_ = false;
    try
    {
      close();
      io_service.stop();
      if (boost::this_thread::get_id() != io_service_thread.get_id())
        io_service_thread.join();
    }
    catch (std::exception &e)
    {
      std::cerr << "Exception: " << e.what() << std::endl;
    }
  }

  void set_handle_read(boost::function<void(DataParser *parser,
                        std::basic_string<unsigned char> str)>
                        h,
                        DataParser *parser)
  {
    handle_read = boost::bind(h, parser, boost::placeholders::_1);
  }

  virtual void connect() = 0;
  virtual void close() = 0;

protected:
  std::basic_string<unsigned char> get_buffer_string(std::size_t n)
  {
    buf.commit(n);
    std::basic_string<unsigned char> s(boost::asio::buffers_begin(buf.data()),
                       boost::asio::buffers_end(buf.data()));
    buf.consume(n);
    return s;
  }

  void handle_packet(const boost::system::error_code &ec, std::size_t n)
  {
    (void)ec;  // Avoid warning "unused parameter"
           // ToDo: Implement error handling for this if needed
    std::basic_string<unsigned char> str = get_buffer_string(n);
    if (str.empty())
      return;

    handle_read(str);
    async_read(65507, &Connection::handle_packet);
  }

  boost::thread io_service_thread;
  boost::asio::io_service io_service;

  boost::asio::streambuf buf;

  boost::function<void(std::basic_string<unsigned char>)> handle_read;
  virtual void async_read(std::size_t s, boost::function<void(Connection *conn,
                                const boost::system::error_code &ec,
                                std::size_t n)>
                                h) = 0;

  bool is_connected_;
  double last_data_time;
  std::string ip_address, port;
};

class UDPConnection : public Connection
{
public:
  UDPConnection(std::string IP, std::string port)
  {
    this->ip_address = IP;
    this->port = port;
  }
  void connect()
  {
    udp_socket = new boost::asio::ip::udp::socket(io_service,
      boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
      atoi(port.c_str())));
    port = std::to_string(udp_socket->local_endpoint().port());
    udp_endpoint = boost::asio::ip::udp::endpoint(
      boost::asio::ip::address::from_string(ip_address),
      atoi(port.c_str()));

    is_connected_ = true;
  }

  void close()
  {
    if (udp_socket)
      udp_socket->close();
  }

private:
  void async_read(std::size_t s, boost::function<void(Connection *conn,
                  const boost::system::error_code &ec, std::size_t n)>
                  handle_packet)
  {
#ifndef SIMULATION
    boost::asio::streambuf::mutable_buffers_type bufs = buf.prepare(s);

    // This is no simulation => Receive data from a real sensor:
    udp_socket->async_receive_from(boost::asio::buffer(bufs), udp_endpoint,
                     boost::bind(handle_packet, this,
                           boost::asio::placeholders::error,
                           boost::asio::placeholders::bytes_transferred));
#else
    // Create a thread for background data generator
    boost::thread thr(&UdpSim::data_generator, handle_read);

    // Let the thread run independently from the main thread:
    thr.detach();
#endif
  }

  boost::asio::ip::udp::socket *udp_socket;
  boost::asio::ip::udp::endpoint udp_endpoint;
};

#endif  // LEUZE_RSL_DRIVER__COMMUNICATION_HPP_
