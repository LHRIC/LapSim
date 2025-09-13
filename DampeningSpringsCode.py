import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import mplcursors
import re
import os

def main():
    # Get file path from user
    file_path = input("Enter CSV file path: ").strip()
    
    # Determine file type (LS or HS)
    if '_LS' in file_path or ' LS' in file_path:
        file_type = 'LS'
    else:
        file_type = 'HS'

    # Load CSV data
    try:
        df = pd.read_csv(file_path, on_bad_lines='skip')
    except Exception as e:
        print(f"Error loading file: {e}")
        return

    # Extract settings from column headers
    setting_values = set()
    pattern = r'\((\d+)-4\.3\) V-' if file_type == 'LS' else r'\(\d+-([\d.]+)\) V-'
    
    for col in df.columns:
        match = re.search(pattern, col)
        if match:
            setting = float(match.group(1))
            setting_values.add(setting)#change

    if not setting_values:
        print("No valid settings found in CSV file")
        return

    available_settings = sorted(setting_values)
    print("Available settings:", [int(s) if s.is_integer() else s for s in available_settings])
    
    # Get user input
    desired_setting = float(input("Enter desired setting: "))
    query_velocity = float(input("Enter velocity to query (mm/sec): "))

    # Create figure
    fig, ax = plt.subplots(figsize=(10, 7))

    # Handle existing setting
    if desired_setting in available_settings:
        # Construct setting string based on file type
        if file_type == 'LS':
            setting_str = f"{int(desired_setting)}-4.3" if desired_setting.is_integer() else f"{desired_setting}-4.3"
        else:
            setting_str = (f"0-{int(desired_setting)}" if desired_setting.is_integer() 
                          else f"0-{desired_setting:.1f}")

        # Extract columns
        cols = {
            'v_comp': f"({setting_str}) V-C",
            'f_comp': f"({setting_str}) C",
            'v_reb': f"({setting_str}) V-R",
            'f_reb': f"({setting_str}) R"
        }

        try:
            comp_vel = df[cols['v_comp']].dropna()
            comp_force = df[cols['f_comp']].dropna()
            reb_vel = df[cols['v_reb']].dropna()
            reb_force = df[cols['f_reb']].dropna()
        except KeyError as e:
            print(f"Missing data columns for setting {setting_str}: {e}")
            return

        # Plot actual data
        ax.plot(comp_vel, comp_force, 'o-', label=f"{setting_str} Compression")
        ax.plot(reb_vel, reb_force, 'o-', label=f"{setting_str} Rebound")

        # Add query points
        comp_query = np.interp(query_velocity, comp_vel, comp_force)
        reb_query = np.interp(query_velocity, reb_vel, reb_force)
        ax.scatter(query_velocity, comp_query, s=100, c='red', label='Queried Compression')
        ax.scatter(query_velocity, reb_query, s=100, c='blue', label='Queried Rebound')

    else:
        # Interpolation logic
        lower_setting = max([s for s in available_settings if s <= desired_setting], default=None)
        upper_setting = min([s for s in available_settings if s >= desired_setting], default=None)

        if None in [lower_setting, upper_setting]:
            print("Error: Desired setting outside available range")
            return

        # Construct setting strings
        def format_setting(s, file_type):
            if file_type == 'LS':
                return f"{int(s)}-4.3" if s.is_integer() else f"{s}-4.3"
            return f"0-{int(s)}" if s.is_integer() else f"0-{s:.1f}"

        lower_str = format_setting(lower_setting, file_type)
        upper_str = format_setting(upper_setting, file_type)
        desired_str = format_setting(desired_setting, file_type)

        # Extract data for both settings
        data = {}
        for setting, suffix in [(lower_str, 'lower'), (upper_str, 'upper')]:
            try:
                data[suffix] = {
                    'v_comp': df[f"({setting}) V-C"].dropna(),
                    'f_comp': df[f"({setting}) C"].dropna(),
                    'v_reb': df[f"({setting}) V-R"].dropna(),
                    'f_reb': df[f"({setting}) R"].dropna()
                }
            except KeyError as e:
                print(f"Missing data for {setting}: {e}")
                return

        # Interpolation calculations
        unified_vel = np.union1d(data['lower']['v_comp'], data['upper']['v_comp'])
        
        if lower_setting == upper_setting:
            weight_lower, weight_upper = 1.0, 0.0
        else:
            weight_lower = (upper_setting - desired_setting) / (upper_setting - lower_setting)
            weight_upper = 1 - weight_lower

        # Interpolate forces
        interp_comp = (weight_lower * np.interp(unified_vel, data['lower']['v_comp'], data['lower']['f_comp']) +
                      weight_upper * np.interp(unified_vel, data['upper']['v_comp'], data['upper']['f_comp']))

        interp_reb = (weight_lower * np.interp(unified_vel, data['lower']['v_reb'], data['lower']['f_reb']) +
                     weight_upper * np.interp(unified_vel, data['upper']['v_reb'], data['upper']['f_reb']))

        # Query points
        comp_query = np.interp(query_velocity, unified_vel, interp_comp)
        reb_query = np.interp(query_velocity, unified_vel, interp_reb)

        # Plotting
        for suffix, style in [('lower', 'o-'), ('upper', 'o-')]:
            ax.plot(data[suffix]['v_comp'], data[suffix]['f_comp'], style, 
                   label=f"{format_setting(lower_setting if suffix=='lower' else upper_setting, file_type)} Compression")
            ax.plot(data[suffix]['v_reb'], data[suffix]['f_reb'], style,
                   label=f"{format_setting(lower_setting if suffix=='lower' else upper_setting, file_type)} Rebound")

        ax.plot(unified_vel, interp_comp, 'x--', label=f"{desired_str} Compression (Interpolated)")
        ax.plot(unified_vel, interp_reb, 'x--', label=f"{desired_str} Rebound (Interpolated)")
        ax.scatter(query_velocity, comp_query, s=100, c='red', label='Queried Compression')
        ax.scatter(query_velocity, reb_query, s=100, c='blue', label='Queried Rebound')

    # Final plot configuration
    ax.set_xlabel('Velocity (mm/sec)')
    ax.set_ylabel('Force (N)')
    title_type = 'LS' if file_type == 'LS' else 'HS'
    ax.set_title(f'Force vs Velocity ({title_type} Data)')

    plt.subplots_adjust(
        left=0.095,     # Left margin
        bottom=0.114,   # Bottom margin
        right=0.716,    # Right margin
        top=0.88,       # Top margin
        wspace=0.2,     # Width spacing between subplots
        hspace=0.2      # Height spacing between subplots
    )


    # Adjust legend placement and font size
    ax.legend(
        bbox_to_anchor=(1.05, 1),  # Position legend outside plot to the right
        loc='upper left',          # Anchor point for the legend
        fontsize='small'           # Adjust font size to ensure readability
    )

    # Save and show plot
    base_name = os.path.splitext(os.path.basename(file_path))[0]
    plt.savefig(f'{base_name}_plot.png', bbox_inches='tight')
    mplcursors.cursor(hover=True)
    plt.show()

if __name__ == "__main__":
    main()
