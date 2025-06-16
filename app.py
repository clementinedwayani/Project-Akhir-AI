from flask import Flask, render_template, redirect, request, url_for
import ast


app = Flask(__name__)

@app.route("/")
def home():
    return render_template('beranda.html')

@app.route("/rute-halte", methods=["POST"])
def rute_halte():
    halte_awal = request.form["halte_awal"]
    halte_tujuan = request.form["halte_tujuan"]
    # Lakukan perhitungan rute terpendek di sini
    return f"Rute dari {halte_awal} ke {halte_tujuan}"

@app.route("/rute-wisata", methods=["POST"])
def rute_wisata():
    halte_asal = request.form["halte_asal"]
    # Hitung tempat wisata terdekat dari halte
    return f"Tempat wisata terdekat dari {halte_asal}"

@app.route("/beranda")
def beranda():
    return render_template("beranda.html")  # Halaman Beranda

@app.route("/tentang")
def about():
    return render_template("tentang.html")  # Halaman Tentang

if __name__ == "__main__":
    app.run(debug=True, port=5001)