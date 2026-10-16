"""Frozen 230-command diagnostic arithmetic; no planning or simulation."""

def generate_actions(initial, plan):
    """Same operation order as the historical helper, no planner invocation."""
    start = initial['arm_absolute_qpos_rad']
    opened = initial['hand_active_source_range_fractions']
    closed = opened[:]
    path = plan['nodes_contact_to_above']
    actions = []
    for macro in range(230):
        if macro == 10:
            opened = plan['open_source_fractions']
            closed = plan['closed_source_fractions']
        if macro < 10:
            phase, arm, hand = 'settle', start, opened
        elif macro < 50:
            phase = 'approach_high'
            alpha = (macro - 10) / 39.
            arm = [(1-alpha)*x + alpha*y for x, y in zip(start, path[-1]['arm_absolute_qpos_rad'])]
            hand = opened
        else:
            if macro < 120:
                phase, coordinate, hand = 'descend', 10*(1-(macro-50)/69.), opened
            elif macro < 160:
                phase, coordinate = 'close', 0.
                alpha = min(1., (macro-120)/30.)
                hand = [(1-alpha)*x + alpha*y for x, y in zip(opened, closed)]
            elif macro < 200:
                phase, coordinate, hand = 'lift', 10*(macro-160)/39., closed
            else:
                phase, coordinate, hand = 'hold', 10., closed
            i = min(9, int(coordinate))
            alpha = coordinate-i
            arm = [(1-alpha)*x + alpha*y for x, y in zip(path[i]['arm_absolute_qpos_rad'], path[i+1]['arm_absolute_qpos_rad'])]
        actions.append({'macro': macro, 'phase': phase, 'action': list(arm) + [2*x-1 for x in hand]})
    return actions
