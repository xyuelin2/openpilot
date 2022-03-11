#!/usr/bin/env python3
import threading
from wsgiref.simple_server import WSGIServer, make_server
from cgi import parse
import capnp

#from cereal import car
import json

# POC / example web service for live tuning
# TODO: auth, use an existing restful library
# TODO: make the code not suck
# TODO: not using a class here makes me feel kinda sick...

httpd : WSGIServer
th : threading.Thread
global _CP
# TODO: May need to do this the right way and create a service with events to be reacted to...

def app(environ, start_response):
    global _CP
    if _CP is None:
        start_response('200 OK', [('Content-Type', 'text/json')])
        return [b"Whoopsie no _CP"]
    
    d = parse(environ['QUERY_STRING'])
    
    if d is not None and len(d) > 0:
        kfraw = d.get('lateralTuning_pid_kf', [''])[0]
        if kfraw is not None:
            kf = float(kfraw)
            _CP.lateralTuning.pid.kf = kf
    
    
    data = json.dumps(capnp_to_json(_CP))
    start_response('200 OK', [('Content-Type', 'text/json')])
    return [data.encode()]
    
    
    # if environ['REQUEST_METHOD'] == 'POST':
    #     start_response('200 OK', [('Content-Type', 'text/json')])
    #     return [b'']


def launch_listener_async(CP):
    global _CP
    #if (httpd is not None):
    #    return
    #started = True
    _CP = CP
    th = threading.Thread(target=_launch_listener)
    th.start()

def _launch_listener():
    #TODO: error handling, etc
    httpd = make_server('', 8282, app)
    httpd.serve_forever()




DYNAMIC_ENUM_MARKER = "_de_"

def capnp_to_json(message):
    class CapnpEncoder(json.JSONEncoder):
        def default(self, obj):
            if isinstance(obj, capnp.lib.capnp._DynamicStructBuilder):
                return obj.to_dict()
            if isinstance(obj, capnp.lib.capnp._DynamicEnum):
                return {DYNAMIC_ENUM_MARKER:str(obj)}
            return json.JSONEncoder.default(self, obj)

    json_string = json.dumps(message, cls=CapnpEncoder)

    return json_string

def json_to_capnp(capnp_message_type, json_string):
    def as_dynamic_enum(dct):
        if DYNAMIC_ENUM_MARKER in dct:
            return capnp.lib.capnp._DynamicEnum(dct[DYNAMIC_ENUM_MARKER])
        return dct

    loaded_dict = json.loads(json_string, object_hook=as_dynamic_enum)

    return capnp_message_type.new_message(**loaded_dict)










# if __name__ == '__main__':
#     try:        
#         launch_listener()
#         print('Serving on port 8282...')
#         httpd.serve_forever()
#     except KeyboardInterrupt:
#         print('Goodbye.')