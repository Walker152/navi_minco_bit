#pragma once
#include <event.h>
#include <event2/bufferevent.h>
#include <event2/listener.h>
#include <event2/thread.h>
#include <event2/util.h>

#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
namespace MyUtils {
namespace Net {
class Server
{
private:
  int m_listener_fd;
  int m_port;
  evconnlistener_cb m_listener_cb;

  evconnlistener * m_listener;
  event_base * m_base;
  struct event * m_signal_event;

public:
  Server() = delete;
  /**
   * @brief 使用libevent实现的高性能服务器
   * @param [in] port 服务器绑定到的端口号
   * @param [in] num_max_connextion 最大允许连接数量
   * @param [in] listener_cb
   * 建立连接需要的回调函数。可以参考本源代码中的回调函数样例进行实现
   */
  explicit Server(int port, int num_max_connextion, evconnlistener_cb listener_cb)
  : m_listener_fd(0), m_port(port), m_listener_cb(listener_cb), m_listener(NULL), m_base(NULL),
    m_signal_event(NULL)
  {
    evthread_use_pthreads();  //会自动添加锁机制
    m_base = event_base_new();
    if (!m_base) {
      m_signal_event = evsignal_new(m_base, SIGINT, signal_cb, (void *)this);
    }
    struct sockaddr_in sin;
    memset(&sin, 0, sizeof(struct sockaddr_in));
    sin.sin_family = AF_INET;
    sin.sin_port = htons(m_port);

    m_listener = evconnlistener_new_bind(m_base,
      m_listener_cb,
      m_base,
      LEV_OPT_REUSEABLE | LEV_OPT_CLOSE_ON_FREE | LEV_OPT_THREADSAFE,
      num_max_connextion,
      (struct sockaddr *)&sin,
      sizeof(struct sockaddr_in));
  }
  ~Server()
  {
    raise(SIGINT);
    if (!m_listener) {
      evconnlistener_free(m_listener);
      m_listener = NULL;
    }
    if (!m_base) {
      event_base_free(m_base);
      m_base = NULL;
    }
  }
  /**
   * @brief 运行服务器。该函数需要放置在一个新的线程中.
   */
  inline void run() { event_base_dispatch(m_base); }

private:
  static void signal_cb(evutil_socket_t sig, short events, void * user_data)
  {
    Server * ths = (Server *)user_data;
    printf("Caught an interrupt signal; exiting cleanly in two seconds.\n");
    evconnlistener_free(ths->m_listener);
    event_free(ths->m_signal_event);
    ths->m_signal_event = NULL;
    event_base_loopbreak(ths->m_base);
    event_base_free(ths->m_base);
    ths->m_base = NULL;
  }
  /******************下面的代码是listenenr_cb的样例***********************/

  //一个新客户端连接上服务器了
  //当此函数被调用时，libevent已经帮我们accept了这个客户端。该客户端的
  //文件描述符为fd
  static void _listener_cb(
    evconnlistener * listener, evutil_socket_t fd, struct sockaddr * sock, int socklen, void * arg)
  {
    printf("accept a client %d\n", fd);
    event_base * base = (event_base *)arg;
    //为这个客户端分配一个bufferevent
    bufferevent * bev = bufferevent_socket_new(base, fd, BEV_OPT_CLOSE_ON_FREE);
    bufferevent_setcb(bev, _socket_read_cb, NULL, _socket_event_cb, NULL);
    bufferevent_enable(bev, EV_READ | EV_PERSIST);
  }

  static void _socket_read_cb(bufferevent * bev, void * arg)
  {
    char msg[4096];
    size_t len = bufferevent_read(bev, msg, sizeof(msg) - 1);
    msg[len] = '\0';
    printf("server read the data %s\n", msg);
    char reply[] = "I has read your data";
    bufferevent_write(bev, reply, strlen(reply));
  }

  static void _socket_event_cb(bufferevent * bev, short events, void * arg)
  {
    if (events & BEV_EVENT_EOF)
      printf("ros_com: connection closed\n");
    else if (events & BEV_EVENT_ERROR)
      printf("ros_com: some other error\n");
    //这将自动close套接字和free读写缓冲区
    bufferevent_free(bev);
  }

  static void _event_cb(struct bufferevent * bev, short event, void * arg)
  {
    if (event & BEV_EVENT_EOF)
      printf("ros_com: connection closed\n");
    else if (event & BEV_EVENT_ERROR)
      printf("ros_com: some other error\n");
    //这将自动close套接字和free读写缓冲区
    bufferevent_free(bev);
  }
};
}  // namespace Net
}  // namespace MyUtils
