import xmlrpc.client


class SimTimeContractError(RuntimeError):
    pass


def require_boolean_sim_time(master_uri, caller_id,
                             proxy_factory=xmlrpc.client.ServerProxy):
    try:
        response = proxy_factory(master_uri).getParam(
            caller_id, "/use_sim_time")
    except Exception as error:
        raise SimTimeContractError(
            "could not read /use_sim_time: %s" % error)
    if not isinstance(response, (list, tuple)) or len(response) != 3:
        raise SimTimeContractError("invalid ROS master getParam response")
    code, message, value = response
    if type(code) is not int or code != 1 or type(message) is not str:
        raise SimTimeContractError("ROS master rejected /use_sim_time")
    if type(value) is not bool or value is not True:
        raise SimTimeContractError("/use_sim_time must be boolean true")
    return True
