#!/usr/bin/env python3
"""
墙面跟随状态机Webview服务器
提供独立的HTTP服务器来展示墙面跟随状态机图片
"""

import os
import sys
import time
import threading
from http.server import HTTPServer, SimpleHTTPRequestHandler
from socketserver import ThreadingMixIn
import argparse
import logging

# 设置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    """支持多线程的HTTP服务器"""
    daemon_threads = True
    allow_reuse_address = True


class WallFollowWebViewHandler(SimpleHTTPRequestHandler):
    """墙面跟随Webview处理器"""
    
    def __init__(self, *args, **kwargs):
        # 设置webview目录为当前目录
        webview_dir = os.path.join(os.path.dirname(__file__), '..', 'webview', 'wall_following')
        webview_dir = os.path.abspath(webview_dir)
        if os.path.exists(webview_dir):
            os.chdir(webview_dir)
        else:
            logger.error(f"Wall following webview directory not found: {webview_dir}")
        super().__init__(*args, **kwargs)
    
    def end_headers(self):
        # 添加CORS头以支持跨域访问
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET, POST, OPTIONS')
        self.send_header('Access-Control-Allow-Headers', 'Content-Type')
        super().end_headers()
    
    def do_GET(self):
        """处理GET请求"""
        try:
            # 如果请求根路径，重定向到webview.html
            if self.path == '/':
                self.path = '/webview.html'
            
            # 调用父类方法处理请求
            return super().do_GET()
        except Exception as e:
            logger.error(f"Error handling GET request: {e}")
            self.send_error(500, f"Internal server error: {e}")
    
    def log_message(self, format, *args):
        """自定义日志格式"""
        logger.info(f"{self.address_string()} - {format % args}")


class WallFollowWebViewServer:
    """墙面跟随Webview服务器"""
    
    def __init__(self, host='127.0.0.1', port=9000):
        self.host = host
        self.port = port
        self.server = None
        self.server_thread = None
        self.running = False
    
    def start(self):
        """启动服务器"""
        try:
            # 创建服务器
            self.server = ThreadedHTTPServer((self.host, self.port), WallFollowWebViewHandler)
            
            # 在单独线程中运行服务器
            self.server_thread = threading.Thread(target=self.server.serve_forever)
            self.server_thread.daemon = True
            self.server_thread.start()
            
            self.running = True
            logger.info(f"Wall following webview server started at http://{self.host}:{self.port}")
            logger.info(f"Access webview at: http://{self.host}:{self.port}/webview.html")
            
        except Exception as e:
            logger.error(f"Failed to start wall following webview server: {e}")
            raise
    
    def stop(self):
        """停止服务器"""
        if self.server and self.running:
            self.server.shutdown()
            self.server.server_close()
            self.running = False
            logger.info("Wall following webview server stopped")
    
    def is_running(self):
        """检查服务器是否正在运行"""
        return self.running


def create_arg_parser():
    """创建命令行参数解析器"""
    parser = argparse.ArgumentParser(description="Wall Following State Machine Webview Server")
    parser.add_argument("--host", default="127.0.0.1", help="服务器主机地址")
    parser.add_argument("--port", type=int, default=9000, help="服务器端口")
    parser.add_argument("--debug", action="store_true", help="启用调试模式")
    return parser


def main():
    """主函数"""
    parser = create_arg_parser()
    args = parser.parse_args()
    
    if args.debug:
        logging.getLogger().setLevel(logging.DEBUG)
    
    # 创建并启动服务器
    server = WallFollowWebViewServer(host=args.host, port=args.port)
    
    try:
        server.start()
        logger.info("Wall following webview server is running. Press Ctrl+C to stop.")
        
        # 保持服务器运行
        while server.is_running():
            time.sleep(1)
            
    except KeyboardInterrupt:
        logger.info("Received interrupt signal, stopping server...")
    except Exception as e:
        logger.error(f"Server error: {e}")
    finally:
        server.stop()


if __name__ == "__main__":
    main()
