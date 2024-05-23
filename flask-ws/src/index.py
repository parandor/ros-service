from flask import Flask, jsonify, request

app = Flask(__name__)

hello = [
  { 'description': 'test_parameter', 'my_parameter': 'from_flask' }
]

@app.route("/hello")

def get_hello_param():
    return jsonify(hello)
