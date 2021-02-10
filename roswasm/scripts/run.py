#!/usr/bin/env python

import tornado.ioloop
import tornado.web
import os
import rospy
import datetime
from catkin.find_in_workspaces import find_in_workspaces

cwd = "" # os.getcwd() # used by static file server
rosbridge_ip = ""
rosbridge_port = ""
html_file = ""

class MainHandler(tornado.web.RequestHandler):
    def get(self):
        self.render(os.path.join(cwd, html_file), rosbridge_ip=rosbridge_ip, rosbridge_port=rosbridge_port)

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
    #cwd = rospy.get_param('~html_dir')
    pkg = rospy.get_param('~pkg')
    node = rospy.get_param('~node')
    checked = []
    results = find_in_workspaces([], pkg, node, first_matching_workspace_only=True, first_match_only=True, considered_paths=checked)
    if len(results) > 1:
        raise RuntimeError('Could not find unique path, the following paths are matching:\n%s' % '\n'.join(results))
    elif len(results) == 0:
        raise RuntimeError('Could not find any path, checked the following paths:\n%s' % '\n'.join(checked))
    html_file = os.path.basename(results[0]) # rospy.get_param('~html_file')
    cwd = os.path.dirname(results[0])
    rospy.loginfo("Found node: %s", results[0])
    rosbridge_ip = rospy.get_param('~rosbridge_ip', '127.0.0.1')
    rosbridge_port = rospy.get_param('~rosbridge_port', '9090')
    display_port = rospy.get_param('~display_port', '8080')
    app = make_app()
    app.listen(int(display_port))
    ioloop = tornado.ioloop.IOLoop.current()
    set_ping(ioloop, datetime.timedelta(seconds=2))
    ioloop.start()
