"""Backend-neutral stop operation for the common ground navigation layer."""


def stop_robot(move_base_client, command_publisher, twist_type):
    """Cancel navigation and deliver one explicit zero command to its output."""
    move_base_client.cancel_all_goals()
    command_publisher.publish(twist_type())
    return {"success": True, "message": "ground stopped"}
