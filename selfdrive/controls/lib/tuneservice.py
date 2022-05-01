#!/usr/bin/env python
# encoding: utf-8
#import cereal
#from cereal import car
import json
from flask import Flask, request, jsonify
import threading
from selfdrive.swaglog import cloudlog

app = Flask(__name__)

_CP = None

def InitTuneServer(CP):
  global _CP
  _CP = CP
  threading.Thread(target=lambda: app.run(host='0.0.0.0', port=5000, debug=True, use_reloader=False)).start()
  #app.run( host='0.0.0.0', port=5000, debug=True, use_reloader=False )

@app.route('/CP', methods=['GET'])
def query_records():
  global _CP
  if _CP is None:
    return jsonify({'error': 'data not found'})
  
  return jsonify(_CP.to_dict())
        

def updateField(chunk, key, value):
  if isinstance(value, dict):
    for subKey in value:
      updateField(getattr(chunk,key),subKey,value[subKey])
  elif isinstance(value, list):
    for x in range(len(value)):
      updateField(getattr(chunk,key),x,value[x])
  else:
    #TODO: May want to determine field type using capnp schema instead...
    cloudlog.info(f"Live Tuner Updating '{key}' to '{value}'")
    if isinstance(key,int):
      chunk[key] = value
    else:
      setattr(chunk,key,value)


@app.route('/CP', methods=['POST'])
def update_record():
  global _CP
  updates = json.loads(request.data)
  for key in updates:
    updateField(_CP,key,updates[key])
    
  return jsonify(_CP.to_dict())
