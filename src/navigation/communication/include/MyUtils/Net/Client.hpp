#pragma once
#include <arpa/inet.h>
#include <event.h>
#include <event2/bufferevent.h>
#include <event2/listener.h>
#include <event2/thread.h>
#include <functional>
#include <future>
#include <netinet/in.h>
#include <stdio.h>
#include <string.h>
#include <string>
#include <sys/socket.h>
#include <unistd.h>

namespace MyUtils {
namespace Net {

class Client
{
private:
public:
  Client();
  ~Client();

public:
  /**
   * @brief 连接到服务器。返回值是建立的连接的fd。如果为-1则表示连接失败
   * @param [in] m_mode 模式选择，SOCK_STREAM是TCP模式，SOCK_DGRAM是UDP模式
   */
  static int connect(const int & m_mode, const int & m_server_port, const char * m_server_ip)
  {
    int sockfd, status, save_errno;
    struct sockaddr_in server_addr;
    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(m_server_port);
    status = inet_aton(m_server_ip, &server_addr.sin_addr);
    if (status == 0)  // the server_ip is not valid value
    {
      errno = EINVAL;
      return -1;
    }
    sockfd = ::socket(PF_INET, SOCK_STREAM, 0);
    if (sockfd == -1)
      return sockfd;
    status = ::connect(sockfd, (struct sockaddr *)&server_addr, sizeof(server_addr));
    if (status == -1) {
      save_errno = errno;
      ::close(sockfd);
      errno = save_errno;  // the close may be error
      return -1;
    }
    evutil_make_socket_nonblocking(sockfd);
    return sockfd;
  }
};

}  // namespace Net
}  // namespace MyUtils
