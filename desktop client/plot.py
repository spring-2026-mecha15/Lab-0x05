from matplotlib import pyplot
from matplotlib.gridspec import GridSpec
import os
import math

############################################################################
# Author: Gabe Haarberg                                                    #
# Date:   Feb 20, 2026                                                     #
# File:   main.py                                                          #
#                                                                          #
# Description:                                                             #
#  A script that plots two columns of data from a local csv file           #
#  using matplotlib. Columns with improperly formatted data are            #
#  handled and alerted to the user via the terminal.                       #
############################################################################

def plot_csv(filenames: list[str]):
    # Create figure with custom layout using GridSpec
    # Top: 1 large combined plot spanning full width
    # Bottom: Individual plots arranged in a grid
    num_files = len(filenames)
    individual_cols = max(1, min(3, math.ceil(math.sqrt(num_files))))
    individual_rows = math.ceil(num_files / individual_cols)
    total_rows = 1 + individual_rows

    fig = pyplot.figure(figsize=(5 * individual_cols, 4 + 3 * individual_rows))
    gs = GridSpec(total_rows, individual_cols, figure=fig, height_ratios=[1.3] + [1] * individual_rows)

    # Combined plot spans all columns on the first row
    ax_combined = fig.add_subplot(gs[0, :])

    # Individual plots fill the remaining grid rows
    axes_individual = []
    for file_idx in range(num_files):
        row = 1 + (file_idx // individual_cols)
        col = file_idx % individual_cols
        axes_individual.append(fig.add_subplot(gs[row, col]))
    
    # Store all data for plotting
    all_data = []
    
    # Setup global arrays
    # col_names = []
    for file_idx, filename in enumerate(filenames):
        data = []

        # Check if file exists
        try:
            open(filename, 'r')
        except FileNotFoundError:
            print(f'No local file found named: {filename}')
            exit()

        with open(filename, 'r') as fhand:
            # print(f'Reading data from local file: {filename}')
            # Read first line as header
            header = next(fhand)
            header = header.split(',') # Convert header string into array separated by commas

            # # Filter undesired elements from header strings
            # for i in range(len(header)):
            #     header[i] = header[i].strip()     # Remove surrounding whitespace

            # try:
            #     # col_names = [str(header[0]), str(header[1])] # We only want first two columns
            #     col_names = [str(header_name).strip() for header_name in header[0:NUM_COLS]] # We only want first two columns
            # except:
            #     print('Error: Could not convert column names into strings!')

            
            # Process each line of text in the file
            for row_num, line in enumerate(fhand):
                try:
                    # Try to split row by commas if present
                    row = line.split(',')

                    row[len(row)-1] = row[len(row)-1].rstrip('\n') # Remove newline from last element in row
                    row[-1] = row[-1].rstrip('\n') # Remove newline from last element in row

                    # Try to convert each comma separated element into a float
                    # Note: float() is NOT sensitive to right or left side whitespace
                    # my_floats = float(row[0]), float(row[1])
                    try:
                        my_floats = list(map(lambda x: float(x), row))
                    except Exception as e:
                        print('Error while mapping floats')
                        print(e)

                    # Check if there are an unneccessary number of elements in row
                    if len(row) > 2:
                        print(f'Info: Unused data in extra column(s) on line {row_num + 1}: {str(row)}')

                # Handle specific exceptions
                except IndexError:
                    print(f'Warn: Not enough data on line {row_num + 1}: {str(row)}')

                except ValueError:
                    print(f'Warn: Not a number on line {row_num + 1}: {str(row)}')

                # Alert unhandled exceptions
                except Exception as e:
                    print(f'Warn: Unhandled exception while processing line {row_num + 1}: {str(row)}')
                    print(f' - Exception: {e}')

                # If no exceptions were present, then append row data to data array
                finally:
                    data.append(my_floats)

        # print(f'Info: Processed {len(data)} lines') # Notify how many lines were processed

        # Prepare x and y arrays
        x_values = [x[0] for x in data]
        y_values = [y[1] for y in data]

        # Store data for this file
        all_data.append({
            'x': x_values,
            'y': y_values,
            'header': header,
            'label': os.path.basename(filename)
        })

    # Plot all data overlayed on the single left subplot
    for file_data in all_data:
        ax_combined.plot(file_data['x'], file_data['y'], label=file_data['label'])
    ax_combined.set_xlabel(all_data[0]['header'][0])
    ax_combined.set_ylabel(all_data[0]['header'][1])
    ax_combined.legend()
    ax_combined.set_title('All Data Overlayed')
    ax_combined.grid(True, alpha=0.3)

    # Plot individual data on the right subplots (stacked)
    for file_idx, file_data in enumerate(all_data):
        ax = axes_individual[file_idx]
        ax.plot(file_data['x'], file_data['y'], label=file_data['label'])
        ax.set_xlabel(file_data['header'][0])
        ax.set_ylabel(file_data['header'][1])
        ax.legend()
        ax.set_title(f'Individual: {file_data["label"]}')
        ax.grid(True, alpha=0.3)

    pyplot.tight_layout()

    directory = os.path.dirname(filenames[0])
    figurename = os.path.join(directory, 'results.png')
    pyplot.savefig(figurename, format='png', dpi=150, bbox_inches='tight')

    # Make the plot visible
    pyplot.show()
