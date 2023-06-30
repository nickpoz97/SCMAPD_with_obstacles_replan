import pygame
import sys
import json
import tkinter as tk
from tkinter import filedialog
import os
import fnmatch

# Set up the grid dimensions
n_rows = 21
n_columns = 35

# Create a Tkinter root window (it won't be shown)
root = tk.Tk()
root.withdraw()

# Open a file dialog and get the selected data directory path
result_file_path = filedialog.askopenfilename(initialdir=os.path.abspath(__file__), title='select result file')
grid_file_path = filedialog.askopenfilename(initialdir=os.path.abspath(__file__), title='select grid file')
obstacles_path = filedialog.askopenfilename(initialdir=os.path.abspath(__file__), title='if you want, select obstacles dir')

if not (result_file_path and grid_file_path):
    print("Results or grid path not selected")
    exit(1)

# Load the JSON data from the selected file
with open(result_file_path, 'r') as f:
    data = json.load(f)

# Extract the agent paths from the JSON data

agentsJson = data['agents']
agents = [agent['path'] for agent in agentsJson]

waypoints = [agent['waypoints'] for agent in agentsJson]
waypoints = [
    [
        (
            wp['coords'],
            'P' if wp['demand'] == 'PICKUP' else 'D',
            agents[aI].index(wp['coords'])
        )            
        for wp in agentWps
    ] for aI, agentWps in enumerate(waypoints)
]

for aI, agentWps in enumerate(waypoints):
    agent = agents[aI]
    agentWps.append((agent[0], 'H', len(agent) - 1))

# Load the grid data from a file
with open(grid_file_path, 'r') as f:
    grid_data = f.readlines()

# Set up the grid dimensions
n_rows = len(grid_data)
n_columns = len(grid_data[0].strip())

# Extract the wall positions from the grid data
walls = []
for row in range(n_rows):
    for column in range(n_columns):
        if grid_data[row][column] == '@':
            walls.append((row, column))

# Extract obstacles positions from the grid data
obstacles = list()
if obstacles_path:
    # Load the JSON data from the selected file
    with open(obstacles_path, 'r') as f:
        obs_data = json.load(f)

        for obs_info in obs_data['obstacles']:
            spawn_time = obs_info['t']
            vanish_time = spawn_time + obs_info['interval']
            for t in range(spawn_time, vanish_time):
                obstacles.append((t, obs_info['pos'] // n_columns, obs_info['pos'] % n_columns))

# Set up the colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
ORANGE = (255, 128, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
LIGHT_BLUE = (125, 125, 255)
GRAY = (128, 128, 128)

# Set up the cell size and margin
cell_size = 25
cell_margin = 1

# Set up the screen dimensions
screen_width = n_columns * (cell_size + cell_margin) + cell_margin
screen_height = n_rows * (cell_size + cell_margin) + cell_margin + 50

# Initialize Pygame
pygame.init()

# Set up the screen
screen = pygame.display.set_mode((screen_width, screen_height))

# Set up the font
font = pygame.font.Font(None, 20)

# Set up the slider
slider_x = cell_margin
slider_y = screen_height - 40
slider_width = screen_width - cell_margin * 2
slider_height = 20

# Set up the timestep
timestep = 0

# find longest path value
max_timestep = len(max(agents, key=len))

def draw_grid():
    # Draw the grid
    for row in range(n_rows):
        for column in range(n_columns):
            # Calculate the coordinates of the top left corner of the cell
            x = column * (cell_size + cell_margin) + cell_margin
            y = row * (cell_size + cell_margin) + cell_margin

            # Check if there is a wall in this cell
            if (row, column) in walls:
                color = GRAY
            else:
                color = WHITE
            if (timestep, row, column) in obstacles:
                color = RED

            # Draw the cell
            pygame.draw.rect(screen, color, (x, y, cell_size, cell_size))

def draw_agents():
    # Draw the agents
    for agent in agents:
        # Calculate the row and column of the agent's position at this timestep
        row,column = agent[timestep] if timestep < len(agent) else agent[-1]

        # Calculate the coordinates of the center of the cell
        x = column * (cell_size + cell_margin) + cell_margin + cell_size // 2
        y = row * (cell_size + cell_margin) + cell_margin + cell_size // 2

        # Draw the agent
        pygame.draw.circle(screen, LIGHT_BLUE , (x,y), cell_size/2)

        # Render the text with current agent index value.
        text = font.render(f"{agents.index(agent)}", True, BLACK)
        text_rect=text.get_rect(center=(x,y))
        screen.blit(text,text_rect)

def draw_waypoints():
    # Draw the waypoints

    for aI, agentWps in enumerate(waypoints):
        for wpI, wp in enumerate(agentWps):
            coord, demand, arrival_timstep = wp

            # already crossed
            if timestep >= arrival_timstep:
                continue

            row,column = coord

            # Calculate the coordinates of the center of the cell
            x = column * (cell_size + cell_margin) + cell_margin + cell_size // 2
            y = row * (cell_size + cell_margin) + cell_margin + cell_size // 2

            label = ""

            # Draw the wp
            if wpI != len(agentWps) - 1:
                pygame.draw.circle(screen, ORANGE , (x,y), cell_size/2)
                label = f"a{aI}w{wpI}"
            elif agents[aI][timestep] != [row,column]:
                pygame.draw.circle(screen, GREEN , (x,y), cell_size/2)
                label = f"h{aI}"

            text = font.render(label, True, BLACK)
            # Render the text with current wp index value.
            text_rect=text.get_rect(center=(x,y))
            screen.blit(text,text_rect)

def draw_slider():
    # Draw the slider background
    pygame.draw.rect(screen, WHITE, (slider_x, slider_y, slider_width, slider_height))

    # Calculate the position of the slider button based on the current timestep
    button_x = slider_x + timestep / (max_timestep -1) * slider_width - slider_height //2

    # Draw the slider button
    pygame.draw.circle(screen,RED,(int(button_x+slider_height//2), int(slider_y+slider_height//2)), slider_height//2)

    # Draw a border around the slider
    border_color = BLACK
    border_width = 2
    pygame.draw.rect(screen, border_color, (slider_x - border_width, slider_y - border_width, slider_width + border_width * 2, slider_height + border_width * 2), border_width)

def draw_text():
    # Render the text with current timestep value.
    text = font.render(f"Timestep: {timestep}", True,BLACK)
    text_rect=text.get_rect(center=(screen_width/2,n_rows*(cell_size+cell_margin)+cell_margin+20))
    screen.blit(text,text_rect)

def update_timestep(pos):
    global timestep

    # Check if pos is within slider range.
    if pos[0] >= slider_x and pos[0] <= slider_x+slider_width:
        # Update timestep based on mouse position.
        timestep = int(round((pos[0]-slider_x)/slider_width*(len(max(agents,key=len))-1)))

# Set up the pop-up dialog state
popup_active = False
popup_row = 0
popup_column = 0

def draw_popup():
    if popup_active:
        # Calculate the coordinates of the top left corner of the cell
        x = popup_column * (cell_size + cell_margin) + cell_margin
        y = popup_row * (cell_size + cell_margin) + cell_margin

        # Draw the pop-up background
        popup_width = 100
        popup_height = 50
        popup_x = x + (cell_size - popup_width) // 2
        popup_y = y - popup_height - cell_margin

        # Adjust the pop-up position if it would be off-screen
        if popup_x < 0:
            popup_x = 0
        elif popup_x + popup_width > screen_width:
            popup_x = screen_width - popup_width
        if popup_y < 0:
            popup_y = y + cell_size + cell_margin

        pygame.draw.rect(screen, WHITE, (popup_x, popup_y, popup_width, popup_height))

        # Draw the pop-up border
        border_color = BLACK
        border_width = 2
        pygame.draw.rect(screen, border_color, (popup_x - border_width, popup_y - border_width, popup_width + border_width * 2, popup_height + border_width * 2), border_width)

        # Draw the pop-up text
        text = f"Row: {popup_row}\nColumn: {popup_column}"
        lines = text.split('\n')
        for i, line in enumerate(lines):
            rendered_text = font.render(line, True, BLACK)
            text_rect = rendered_text.get_rect(center=(popup_x + popup_width // 2, popup_y + (i + 1) * (popup_height // (len(lines) + 1))))
            screen.blit(rendered_text, text_rect)


while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            sys.exit()
        elif event.type == pygame.MOUSEBUTTONDOWN:
            pos = pygame.mouse.get_pos()

            # Check if the click was within the grid area
            if pos[0] >= cell_margin and pos[0] < screen_width - cell_margin and pos[1] >= cell_margin and pos[1] < screen_height - cell_margin - 50:
                # Calculate the row and column of the clicked cell
                row = (pos[1] - cell_margin) // (cell_size + cell_margin)
                column = (pos[0] - cell_margin) // (cell_size + cell_margin)

                # Toggle the pop-up dialog state
                if not popup_active or row != popup_row or column != popup_column:
                    # Activate the pop-up dialog and update its position
                    popup_active = True
                    popup_row = row
                    popup_column = column
                else:
                    # Deactivate the pop-up dialog
                    popup_active = False
            else:
                update_timestep(pygame.mouse.get_pos())
        elif event.type == pygame.MOUSEMOTION:
            if pygame.mouse.get_pressed()[0]:
                update_timestep(pygame.mouse.get_pos())
        elif event.type == pygame.KEYDOWN:
            if event.key == pygame.K_LEFT:
                timestep = max(0, timestep - 1)
            elif event.key == pygame.K_RIGHT:
                timestep = min(max_timestep - 1, timestep + 1)

    screen.fill(BLACK)

    draw_grid()
    draw_waypoints()
    draw_agents()
    draw_slider()
    draw_text()
    draw_popup()

    pygame.display.flip()
