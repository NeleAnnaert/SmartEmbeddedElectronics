import sys
import time

if sys.version_info[0] < 3:
    import Tkinter as tk
    import ttk as ttk
    import tkMessageBox as msg
else:
    import tkinter as tk
    from tkinter import ttk
    import tkinter.messagebox as msg


class GUI(tk.Frame):
    SIZE_GRID = 7
    OFFSET = 5

    def __init__(self, mapper, motor_up=None, motor_down=None, motor_left=None, motor_right=None, motor_angle_left=None,
                 motor_angle_right=None, button_1=None, button_2=None, button_3=None):
        if not motor_up:
            motor_up = GUI.print_command
        if not motor_down:
            motor_down = GUI.print_command
        if not motor_left:
            motor_left = GUI.print_command
        if not motor_right:
            motor_right = GUI.print_command
        if not motor_angle_left:
            motor_angle_left = GUI.print_command
        if not motor_angle_right:
            motor_angle_right = GUI.print_command
        if not button_3:
            button_3 = GUI.print_command

        self.root = tk.Tk()
        super().__init__(self.root)
        title = 'Mapper'
        self.root.title(title)

        self.mapper = mapper
        self.mapper.add_observer(self.grid_kleur_in)

        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack()

        self.frame_1 = ttk.Frame(self.notebook)
        self.frame_2 = ttk.Frame(self.notebook)
        self.frame_motor = ttk.Frame(self.notebook)

        self.notebook.add(self.frame_1, text="Mapper")
        self.notebook.add(self.frame_2, text="Camera")
        self.notebook.add(self.frame_motor, text="Motor")

        self.label_location = ttk.Label(self.frame_1, text="Hover over grid", width=50, anchor=tk.CENTER)
        max_height = self.mapper.width * self.SIZE_GRID
        max_width = self.mapper.height * self.SIZE_GRID
        self.canvas = tk.Canvas(self.frame_1, height=max_width + self.OFFSET * 2, width=max_height + self.OFFSET * 2)
        self.rect_list = []
        for row in range(self.mapper.height):
            list = []
            for col in range(self.mapper.width):
                x_1 = col * self.SIZE_GRID + self.OFFSET
                x_2 = (col + 1) * self.SIZE_GRID + self.OFFSET
                y_1 = row * self.SIZE_GRID + self.OFFSET
                y_2 = (row + 1) * self.SIZE_GRID + self.OFFSET
                rect = self.canvas.create_rectangle(x_1, y_1, x_2, y_2)
                list.append(rect)
            self.rect_list.append(list)
        row = 0
        self.canvas.bind("<Motion>", self.mouse_over_canvas)
        self.canvas.bind("<B1-Motion>", self.mouse_pressed_over_canvas)
        self.canvas.bind("<Leave>", self.close_mouse_over)
        self.canvas.bind("<Button-1>", self.canvas_click)

        self.canvas.grid(row=row, column=0, rowspan=20)
        self.label_location.grid(row=row, column=1)
        row += 1

        self.label_x = ttk.LabelFrame(self.frame_1, text="Location x", width=50)
        self.input_x = ttk.Entry(self.label_x)

        self.label_y = ttk.LabelFrame(self.frame_1, text="Location y", width=50)
        self.input_y = ttk.Entry(self.label_y)

        self.label_angle = ttk.LabelFrame(self.frame_1, text="Location angle", width=50)
        self.input_angle = ttk.Entry(self.label_angle)

        self.label_item = ttk.LabelFrame(self.frame_1, text="Location item", width=50)
        items = ["Current location", "Previous location", 
                 "Wall", "Nothing"]
        self.input_item_val = tk.StringVar(self.frame_1)
        self.input_item_val.set(items[-1])
        self.input_item = tk.OptionMenu(self.label_item, self.input_item_val, *items)

        self.label_x.grid(row=row, column=1)
        self.input_x.grid(padx=10, pady=10)
        row += 1
        self.label_y.grid(row=row, column=1)
        self.input_y.grid(padx=10, pady=10)
        row += 1
        self.label_angle.grid(row=row, column=1)
        self.input_angle.grid(padx=10, pady=10)
        row += 1
        self.label_item.grid(row=row, column=1)
        self.input_item.grid(padx=10, pady=10)
        row += 1

        self.update_button = ttk.Button(self.frame_1, text="Update", command=self.update_item)
        self.update_button.grid(row=row, column=1)
        row += 1

        self.in_paint_modus = False
        self.paint_button = ttk.Button(self.frame_1, text="Paint modus", command=self.paint_modus)
        self.paint_button.grid(row=row, column=1)
        row += 1

        self.send_button = ttk.Button(self.frame_1, text="Toggle mapper", command=self.send_command)
        self.send_button.grid(row=row, column=1)
        row += 1

        self.label_weight = ttk.LabelFrame(self.frame_1, text="Weight", width=50)
        self.weight = tk.StringVar()
        self.weight.trace("w", lambda name, index, mode, : self.weight_change())
        self.weight.set(self.mapper.weight)
        self.input_weight = tk.Entry(self.label_weight, textvariable=self.weight)
        self.label_weight.grid(row=row, column=1)
        self.input_weight.grid(padx=10, pady=10)
        row += 1

        self.path_button = ttk.Button(self.frame_1, text="Path plan", command=self.path_plan)
        self.path_button.grid(row=row, column=1)
        row += 1

        self.button_3 = ttk.Button(self.frame_2, text="Position calibration", command=button_3)
        self.button_3.pack()

        self.button_up = ttk.Button(self.frame_motor, text="Omhoog", command=motor_up)
        self.button_down = ttk.Button(self.frame_motor, text="Omlaag", command=motor_down)
        self.button_left = ttk.Button(self.frame_motor, text="Links", command=motor_left)
        self.button_right = ttk.Button(self.frame_motor, text="Rechts", command=motor_right)
        self.button_angle_left = ttk.Button(self.frame_motor, text="Draai links", command=motor_angle_left)
        self.button_angle_right = ttk.Button(self.frame_motor, text="Draai rechts", command=motor_angle_right)
        self.button_up.pack(side=tk.TOP)
        self.button_down.pack(side=tk.BOTTOM)
        self.button_left.pack(side=tk.LEFT)
        self.button_angle_left.pack(side=tk.LEFT)
        self.button_right.pack(side=tk.RIGHT)
        self.button_angle_right.pack(side=tk.RIGHT)

        self.root.bind('z', motor_up)
        self.root.bind('s', motor_down)
        self.root.bind('q', motor_left)
        self.root.bind('d', motor_right)
        self.root.bind('a', motor_angle_left)
        self.root.bind('e', motor_angle_right)

        self.root.bind('<Up>', motor_up)
        self.root.bind('<Left>', motor_left)
        self.root.bind('<Right>', motor_right)
        self.root.bind('<Down>', motor_down)

    def mouse_over_canvas(self, event):
        x, y, item = self.mouse_event_to_grid(event)
        if 0 <= x < self.mapper.width and 0 <= y < self.mapper.height:
            if item == "Wall":
                item += " " + str(self.mapper.map_matrix[y][x] + self.mapper.DELETE_TIME - int(time.time()))
            string = str(x) + "," + str(y) + ": " + item
            self.label_location["text"] = string
        else:
            self.close_mouse_over(None)

    def mouse_pressed_over_canvas(self, event):
        x, y, item = self.mouse_event_to_grid(event)
        if 0 <= x < self.mapper.width and 0 <= y < self.mapper.height:
            if item == "Wall":
                item += " " + str(self.mapper.map_matrix[y][x] + self.mapper.DELETE_TIME - int(time.time()))
            string = str(x) + "," + str(y) + ": " + item
            self.label_location["text"] = string
            if self.in_paint_modus:
                item_menu = str(self.input_item.cget("text"))
                loc = False
                old_y = 0
                old_x = 0
                kleur = "white"
                if item_menu == "Current location":
                    old_x = self.mapper.current_loc.x
                    old_y = self.mapper.current_loc.y
                    self.mapper.current_loc.x = x
                    self.mapper.current_loc.y = y
                    loc = True
                elif item_menu == "Previous location":
                    old_x = self.mapper.previous_loc.x
                    old_y = self.mapper.previous_loc.y
                    self.mapper.previous_loc.x = x
                    self.mapper.previous_loc.y = y
                    loc = True
                elif item_menu == "Wall":
                    current_time = int(time.time())
                    self.mapper.map_matrix[y][x] = current_time
                else:
                    self.mapper.map_matrix[y][x] = 0

        else:
            self.close_mouse_over(None)

    def close_mouse_over(self, event):
        self.label_location["text"] = "Hover over grid"

    def update_item(self):
        # self.root.config(cursor="wait")
        self.root.update()
        x = self.input_x.get()
        y = self.input_y.get()
        angle = self.input_angle.get()
        try:
            x = int(x)
        except ValueError:
            msg.showerror("Ongeldige x waarde", "De waarde voor x is geen geldig getal!")
        else:
            try:
                y = int(y)
            except ValueError:
                msg.showerror("Ongeldige y waarde", "De waarde voor y is geen geldig getal!")
            else:
                try:
                    angle = int(angle)
                except ValueError:
                    msg.showerror("Ongeldige hoek waarde", "De waarde voor de hoek is geen geldig getal!")
                else:
                    if 0 <= x < self.mapper.width:
                        if 0 <= y < self.mapper.height:
                            item = str(self.input_item.cget("text"))
                            if item == "Current location":
                                self.mapper.previous_loc.x = self.mapper.current_loc.x
                                self.mapper.previous_loc.y = self.mapper.current_loc.y
                                self.mapper.previous_loc.angle = self.mapper.current_loc.angle

                                self.mapper.current_loc.x = x
                                self.mapper.current_loc.y = y
                                self.mapper.current_loc.angle = angle
                                self.mapper.changes_loc = True
                            elif item == "Previous location":
                                self.mapper.previous_loc.x = x
                                self.mapper.previous_loc.y = y
                                self.mapper.previous_loc.angle = angle
                                self.mapper.changes_loc = True
                            elif item == "Wall":
                                current_time = int(time.time())
                                self.mapper.place_map(y, x, current_time)
                                self.mapper.changes_obj = True
                            elif item == "Nothing":
                                self.mapper.place_map(y, x, 0)
                                self.mapper.changes_obj = True
                            else:
                                # self.root.config(cursor="arrow")
                                return False
                            self.mapper.update()
                            # self.root.config(cursor="heart")
                            return True
                        else:
                            msg.showerror("Ongeldige y waarde",
                                          "De waarde voor y met tussen %d en %d liggen!" % (0, self.mapper.height - 1))
                    else:
                        msg.showerror("Ongeldige x waarde",
                                      "De waarde voor x met tussen %d en %d liggen!" % (0, self.mapper.width - 1))
        # self.root.config(cursor="arrow")
        return False

    def canvas_click(self, event):
        if not self.in_paint_modus:
            x, y, item = self.mouse_event_to_grid(event)
            if item == "Path":
                item = "Nothing"
            if 0 <= x < self.mapper.width and 0 <= y < self.mapper.height:
                self.input_x.delete(0, tk.END)
                self.input_x.insert(0, str(x))
                self.input_y.delete(0, tk.END)
                self.input_y.insert(0, str(y))
                self.input_item_val.set(item)
                angle = 0
                if item == "Current location":
                    angle = self.mapper.current_loc.angle
                elif item == "Previous location":
                    angle = self.mapper.previous_loc.angle
                elif item == "Current parking spot":
                    angle = self.mapper.parking_loc.angle
                elif item == "Previous parking spot":
                    angle = self.mapper.previous_park.angle
                self.input_angle.delete(0, tk.END)
                self.input_angle.insert(0, str(angle))

    def mouse_event_to_grid(self, event):
        x = int((event.x - self.OFFSET) / self.SIZE_GRID)
        y = int((event.y - self.OFFSET) / self.SIZE_GRID)
        item = ""
        if 0 <= x < self.mapper.width and 0 <= y < self.mapper.height:
            if self.mapper.current_loc.x == x and self.mapper.current_loc.y == y:
                item = "Current location"  # Yellow
            elif self.mapper.previous_loc.x == x and self.mapper.previous_loc.y == y:
                item = "Previous location"  # light green
            elif self.mapper.parking_loc.x == x and self.mapper.parking_loc.y == y:
                item = "Current parking spot"  # dark green
            elif self.mapper.previous_park.x == x and self.mapper.previous_park.y == y:
                item = "Previous parking spot"  # light blue
            elif self.mapper.path_found and ((x, y) in self.mapper.path_found):
                item = "Path"
            elif self.mapper.map_matrix[y][x] == self.mapper.PARKING_WALL:
                item = "Parking wall"
            elif self.mapper.map_matrix[y][x] != 0:
                item = "Wall"
            else:
                item = "Nothing"
        return x, y, item

    def weight_change(self):
        weight = self.weight.get()
        if weight and weight != "-":
            try:
                weight = int(weight)
            except ValueError:
                msg.showerror("Ongeldige waarde gewicht",
                              "Dit is geen geledige waarde voor gewicht")
            else:
                self.mapper.weight = weight
        else:
            self.mapper.weight = 0

    def path_plan(self):
        # self.root.config(cursor="wait")
        self.root.update()
        self.mapper.path_plan()
        # self.root.config(cursor="arrow")

    def paint_modus(self):
        if self.in_paint_modus:
            # self.root.config(cursor="wait")
            self.root.update()
            self.update_button.state(["!disabled"])
            self.path_button.state(["!disabled"])
            self.paint_button['text'] = "Paint modus"
            self.mapper.changes_parking = True
            self.mapper.changes_obj = True
            self.mapper.changes_loc = True
            self.mapper.update()
            # self.root.config(cursor="arrow")
            self.in_paint_modus = False
        else:
            # self.root.config(cursor="spraycan")
            self.root.update()
            self.update_button.state(["disabled"])
            self.path_button.state(["disabled"])
            self.paint_button['text'] = "Klik modus"

            self.in_paint_modus = True

    def send_command(self):
        if self.mapper.send_command:
            self.mapper.send_command = False
            msg.showinfo("Mapper status", "De mapper is uitgeschakeld")
        else:
            self.mapper.send_command = True
            msg.showinfo("Mapper status", "De mapper is ingeschakeld")

    @staticmethod
    def print_command(*args, **kwargs):
        print("Command")
        print(args)
        print(kwargs)

    def quit(self):
exit(10)
