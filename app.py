# app.py
from data import arr
from data import arr2
from flask import Flask, render_template

app = Flask(__name__)

@app.route('/')
def index():
    chart_data = arr[50:150]
    chart_labels = []
    for i in range(50,150):
        chart_labels.append(i)
    heartRate = int(sum(arr)/600.0)
    return render_template('index.html', chart_data=chart_data, chart_labels=chart_labels, name=arr2[0], age=arr2[1], rate=heartRate)

@app.route('/see-graph')
def indexx():
    chart_data = arr[50:350]
    chart_labels = []
    for i in range(50,350):
        chart_labels.append(i)
    
    return render_template('index2.html', chart_data=chart_data, chart_labels=chart_labels)

if __name__ == '__main__':
    app.run(debug=True)
