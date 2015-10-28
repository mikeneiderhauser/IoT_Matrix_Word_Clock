#! /usr/bin/python
# A quick python utility to aid in generating
# bitmaps for the IoT Matrix Word Clock

from Tkinter import *

top = Tk()
# Matrix size
num_rows = 16
num_cols = 16
buttons = {}

# button state colors
false_color = "red"
true_color = "green"


# Function to read state of matrix and export array bitmap
def export_bin():
    print "Printing Matrix"
    print "As seen on screen"
    lines = {}
    for r in range(num_rows):
        line = ""
        for c in range(num_cols):
            tmp_r = num_rows - 1 - r
            tmp_c = num_cols - 1 - c
            bid = (tmp_r*num_cols) + tmp_c
            btn_fields = buttons[bid]
            btn = buttons[bid]["tkid"]
            if tmp_c == 15:
                line = line + "0b"

            if tmp_c == 7:
                line = line + " 0b"

            if buttons[bid]["toggle_state"]:
                line = line + "1"
            else:
                line = line + "0"
        lines[tmp_r] = line
        print line

    print "As code formatable array"
    line = ""
    for r in range(num_rows):
        parts = lines[r].split(' ')
        line = line + parts[1] + "," + parts[0] + ","
    line = line[:-1]
    line = "{" + line + "}"
    print line
    print ""


# clear matrix to off state
def reset_grid():
    for i in range(num_rows*num_cols):
        btn_fields = buttons[i]
        btn = buttons[i]["tkid"]
        btn_fields["toggle_state"] = False
        set_button_color(i)


# helper function to set button color
def set_button_color(bid):
    btn_fields = buttons[bid]
    btn = buttons[bid]["tkid"]

    if btn_fields["toggle_state"]:
        btn.config(bg=true_color)
    else:
        btn.config(bg=false_color)


# Button callback processing
def button_callback(bid):
    btn_fields = buttons[bid]
    btn = buttons[bid]["tkid"]

    # toggle button stat on click
    btn_fields["toggle_state"] = not btn_fields["toggle_state"]

    # update button color
    set_button_color(bid)


# Button callback function wrapper
def action(bid):
    def inneraction():
        button_callback(bid)
    return inneraction


# main
try:
    # import file
    file_name = str(num_rows) + "x" + str(num_cols) + ".cfg"
    with open(file_name) as f:
        for line in f:
            line = line.rstrip('\n')
            parts = line.split(',')
            tmp_r = int(parts[0])
            for c in range(num_cols):
                tmp_c = num_cols - 1 - c
                bid = (tmp_r*num_cols) + tmp_c
                buttons[bid] = {"val": parts[c+1]}

    # generate UI
    for r in range(num_rows):
        for c in range(num_cols):
            tmp_r = num_rows - 1 - r
            tmp_c = num_cols - 1 - c
            bid = (tmp_r*num_cols) + tmp_c
            #n = str(tmp_r) + "," + str(tmp_c)
            n = buttons[bid]["val"]
            b = Button(top, text=n, command=action(bid), height=1, width=1)
            b.grid(row=r, column=c)

            buttons[bid] = {"txt": n, "tkid": b, "toggle_state": False}
            set_button_color(bid)

    # create menu bar
    menubar = Menu(top)
    # add commands to menu bar
    menubar.add_command(label="Export", command=export_bin)
    menubar.add_command(label="Reset", command=reset_grid)
    menubar.add_command(label="Exit", command=top.quit)
    # configure window to add menu
    top.config(menu=menubar)

    top.mainloop()
except KeyboardInterrupt:
    # get rid of (ctrl+c) keyboard interrupts
    top.mainloop()
