from flask import Flask, render_template, redirect, request, url_for
from typing import List, Dict, Tuple
import ast


app = Flask(__name__)

@app.route("/")
def home():
    return render_template('index.html')

if __name__ == "__main__":
    app.run(debug=True)