import json
import os
from random import gauss
from tkinter import Tk, Label, Entry, Button, filedialog

def generate_obstacles(distributions, nRows, nCols):
    obs_short = []
    probs_short = []

    obs_long = []
    probs_long = []

    obs_smart = []
    probs_smart = []

    choices = []

    def add_choice():
        row = int(row_entry.get())
        col = int(col_entry.get())
        t = int(t_entry.get())
        hole = bool(int(hole_entry.get()))
        choices.append({"row": row, "col": col, "t": t, "hole": hole})
        row_entry.delete(0, 'end')
        col_entry.delete(0, 'end')
        t_entry.delete(0, 'end')
        hole_entry.delete(0, 'end')

    def generate():
        for choice in choices:
            row, col, t, hole = choice["row"], choice["col"], choice["t"], choice["hole"]
            pos = row * nCols + col
            
            smart_dist = distributions["long"] if hole else distributions["short"]
            interval = int(gauss(*smart_dist))
            obs_smart.append({"t": t, "pos": pos, "interval": interval})
            probs_smart.append({"pos":pos, "mu":smart_dist[0],"sigma": smart_dist[1]})

            obs_short.append({"t": t, "pos": pos, "interval": int(gauss(*distributions["short"]))})
            probs_short.append({"pos":pos, "mu":distributions["short"][0], "sigma": distributions["short"][1]})

            obs_long.append({"t": t, "pos": pos, "interval": int(gauss(*distributions["long"]))})
            probs_long.append({"pos":pos, "mu":distributions["long"][0], "sigma": distributions["long"][1]})

        save_dir = filedialog.askdirectory(title="Select directory to save JSON files")

        indent_depth = 4

        with open(os.path.join(save_dir, "obs_short.json"), "w") as f:
            json.dump({"obstacles": obs_short, "probability":probs_short}, f, indent=indent_depth)
        with open(os.path.join(save_dir, "obs_long.json"), "w") as f:
            json.dump({"obstacles": obs_long, "probability":probs_long}, f, indent=indent_depth)
        with open(os.path.join(save_dir, "obs_smart.json"), "w") as f:
            json.dump({"obstacles": obs_smart, "probability":probs_smart}, f, indent=indent_depth)
        root.destroy()

    root = Tk()
    root.title("Generate Obstacles")
    Label(root, text="Row:").grid(row=0, column=0)
    row_entry = Entry(root)
    row_entry.grid(row=0,column=1)
    Label(root,text="Column:").grid(row=1,column=0)
    col_entry = Entry(root)
    col_entry.grid(row=1,column=1)
    Label(root,text="T:").grid(row=2,column=0)
    t_entry = Entry(root)
    t_entry.grid(row=2,column=1)
    Label(root,text="Hole (0 or 1):").grid(row=3,column=0)
    hole_entry = Entry(root)
    hole_entry.grid(row=3,column=1)
    Button(root,text="Add Choice",command=add_choice).grid(row=4,column=0,columnspan=2)
    Button(root,text="Generate",command=generate).grid(row=5,column=0,columnspan=2)

    root.mainloop()

# Example usage
distributions = {"short": (4, 1), "long": (50, 5)}
nRows = 21
nCols = 35
generate_obstacles(distributions,nRows,nCols)
