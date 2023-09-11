import matplotlib.pyplot as plt
import numpy as np
import os
import json
import pandas as pd
import random
import datetime
import matplotlib
if "__main__" == __name__:
    os.chdir("../profilingData")
    print(os.listdir())
    files = os.listdir()
    # remove all non json files
    for file in files:
        if file[-4:] != 'json':
            files.remove(file)
    final_data = {

    }
    for file in files:
        data = {}
        with open(file) as f:
            data = json.load(f)
        if data['id'] in final_data:
            final_data[data['id']].append(data)
        else:
            final_data[data['id']] = [data]
        
    for key,value in final_data.items():
        print(str(key)+":"+str(final_data[key][0]['name']))

    # which_test = int(input("Which test would you like to see? "))

    for keys,data in final_data.items():
        a = 1
        ndata = sorted(data, key=lambda k: k['time'],reverse=True)
        for values in ndata:
            print(values['time'])
            x = []
            y = []
            for i in values['dataPoints']:
                y.append(i['values'][0])
                x.append(i['values'][1])
            # 2023-09-09T21:54:36.295162
            date = str(datetime.datetime.strptime(values['time'], "%Y-%m-%dT%H:%M:%S.%f").date())
            df=pd.DataFrame({values['units'][1]:x,date: y})
            # make it lighter the newer it is
            color = matplotlib.colors.to_hex((0.5,0,0,a),keep_alpha=True)
            plt.plot(values['units'][1],date,data=df,marker='o',color=color,linewidth=2)
            a -= 0.9/(len(ndata))
        # plt.plot(x,y)
        plt.ylabel(values['units'][0])
        plt.xlabel(values['units'][1])
        plt.title(values['name'])

        plt.show()
    


