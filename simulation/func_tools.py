# tools need to run AES
# eventually we can put it in a class once we figure out how it will all interact with each other

def get_data(build_file_name, wind_file_name, ts, horiz_len):
    """
    Wrapper to get input data.

    Args:
        build_file_name: file name of building data
        wind_file_name: file name of wind data
        ts: current time-step
        horiz_len: length of prediction horizon

    Returns:
        list: A list of lists of the data.
    """
    build_data = get_building_data(build_file_name, ts, horiz_len)
    wind_data = get_wind_data(wind_file_name, ts, horiz_len)
    grid_data = get_grid_data()

    return [build_data, wind_data, grid_data]


def get_building_data(file_name, ts, horiz_len):
    """
    Get the building data at a certain time-step, returning
    an array(s) of data.

    Args:
        file_name: file name of building data file
        ts: current time-step (could be done with just index?)
        horiz_len: how many time steps to return

    Returns:
        list: A list of lists containing the relevant data,
            [T_oa, Q_int, Q_solar].
    """
    # TODO: Jen - fill in code to get values from building file.
    # General flow?
    #   read-in building data file
    #   grab the respective columns

    # return [[T_oa[ts:ts+horiz]], [Q_int[ts:ts+horiz]], [Q_solar[ts:ts+horiz]]]


def get_wind_data(file_name, ts, horiz_len):
    """
    Get the wind data at a certain time-step, returning
    an array(s) of data.

    Args:
        file_name: file name of wind data file
        ts: current time-step (could be done with just index?)
        horiz_len: how many time steps to return

    Returns:
        list: A list of lists containing the relevant data,
            [Ws, Wd].
    """
    # TODO: Jen - fill in code to get values from wind file.
    # General flow?
    #   read-in wind data file
    #   grab the respective columns

    # return [[Ws[ts:ts+horiz]], [Wd[ts:ts+horiz]]]


def get_grid_data():
    """
    Get the power and voltages from the nodes in openDSS to use as
    linearization points for the voltage constraints. I think just
    grabbing them wholesale works, and we can parse them elsewhere.

    Returns:
        list: A list of lists with the power and voltages,
            [P, V].
    """

    # TODO: Jen - fill in code to get values from openDSS.

    # return [[P], [V]]


def set_truth_inputs(ms_dot, T_sa, yaw, Ct, node_powers):
    """
    Wrapper for the other setting functions for truth models and openDSS.

    Args:
        ms_dot: mass air flow rate driven by the AHU fan
        T_sa: supply air temperature (Celsius)
        yaw: array of yaw values for turbines in wind farm
        Ct: array of Ct values for turbines in wind farm
        node_powers: A dictionary? Containing pairs of node IDs and powers?
    """
    # TODO: Set this up to loop through buildings
    build_powers = set_building_truth(ms_dot, T_sa)
    wind_powers = set_wind_truth(yaw, Ct)
    set_grid_powers([build_powers, wind_powers])


def set_building_truth(ms_dot, T_sa):
    """
    Assign control actions to truth model. It will also need inputs
    from the data files.

    Args:
        ms_dot: mass air flow rate driven by the AHU fan
        T_sa: supply air temperature (Celsius)
    """

    # TODO: Jen - set control actions in truth model; also figure out
    #             how to set other model input from data file:
    #             T_oa, Q_int, and Q_solar.


def set_wind_truth(yaw, Ct):
    """
    Assign control actions to truth model. It will also need inputs
    from the data files.

    Args:
        yaw: array of yaw values for turbines in wind farm
        Ct: array of Ct values for turbines in wind farm
    """

    # TODO: Jen - set control actions in truth model; also figure out
    #             how to set other model input from data file:
    #             Ws and Wd.


def set_grid_powers(node_powers):
    """
    Set the node powers in openDSS so it cn calculate the voltages.

    Args:
        node_powers: A dictionary? Containing pairs of node IDs and powers?
    """

    # TODO: Jen - write code to set node powers in openDSS.