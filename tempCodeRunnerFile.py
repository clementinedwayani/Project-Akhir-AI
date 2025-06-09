from flask import Flask, render_template, redirect, request, url_for
import ast


app = Flask(__name__)

@app.route("/")
def home():
    return render_template('index2.html')

@app.route("/tentang")
def about():
    return render_template("tentang.html")  # Halaman Tentang

if __name__ == "__main__":
    app.run(debug=True)