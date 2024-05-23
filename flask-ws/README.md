A sample Flask webserver. 

1) Please setup a python3 virtualenv environment and pip3 install flask within it. 
2) To run, use the following command: `flask --app index run`

You should be able to hit the webserver through a browser or curl command. 

Example output: 

From curl: 
[{"description":"test_parameter","my_parameter":"From Flask"}]

From browser: 
[
  {
    "description": "test_parameter",
    "my_parameter": "From Flask"
  }
]

