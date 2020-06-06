from __future__ import print_function
import time

from flask import Flask
from flask import render_template

app = Flask(__name__)

@app.route("/")
def index(name=None):
    return render_template("thing.html")

if __name__ == "__main__":
    app.run(host="0.0.0.0", debug=True)