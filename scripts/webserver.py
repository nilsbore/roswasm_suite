#!/usr/bin/python3

import tornado.ioloop
import tornado.web
import os
import rospy
import datetime

cwd = os.getcwd() # used by static file server
rosbridge_ip = ""
rosbridge_port = ""

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render(os.path.join(cwd, "imgui.html"), rosbridge_ip=rosbridge_ip, rosbridge_port=rosbridge_port)

class NoCacheStaticFileHandler(tornado.web.StaticFileHandler):
    def set_extra_headers(self, path):
        # Disable cache
        print("Got asked for template")
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

class WasmHandler(tornado.web.StaticFileHandler):
    def get_content_type(self):
        print("Got asked for mimetype")
        return "application/wasm"

    def set_extra_headers(self, path):
        # Disable cache
        self.set_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')

def make_app():
    return tornado.web.Application([
        (r"/", MainHandler),
        #(r"/*", tornado.web.StaticFileHandler, {"path": cwd }),
        #(r"/(.*\.js)", tornado.web.StaticFileHandler, {"path": cwd }),
        (r"/(.*\.js)", NoCacheStaticFileHandler, {"path": cwd }),
        (r"/(.*\.wasm)", WasmHandler, {"path": cwd }),
        #(r"/(.*\.wasm)", WasmHandler2, {"path": cwd }),
        (r"/(.*\.data)", NoCacheStaticFileHandler, {"path": cwd }),
        (r"/(.*\.ico)", NoCacheStaticFileHandler, {"path": cwd }),
    ], debug=True, static_hash_cache=False)

def set_ping(ioloop, timeout):
    if rospy.is_shutdown():
        ioloop.stop()
    else:
        ioloop.add_timeout(timeout, lambda: set_ping(ioloop, timeout))

if __name__ == "__main__":
    rospy.init_node("wasmros_webserver", anonymous=True)
    cwd = rospy.get_param('~html_dir')
    rosbridge_ip = rospy.get_param('~rosbridge_ip', '127.0.0.1')
    rosbridge_port = rospy.get_param('~rosbridge_port', '9090')
    display_port = rospy.get_param('~display_port', '8081')
    app = make_app()
    app.listen(int(display_port))
    ioloop = tornado.ioloop.IOLoop.current()
    set_ping(ioloop, datetime.timedelta(seconds=2))
    ioloop.start()
