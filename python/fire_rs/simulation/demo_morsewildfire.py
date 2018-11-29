from datetime import datetime

import fire_rs.firemodel.propagation
import fire_rs.simulation.morse

if __name__ == "__main__":
    area = ((480060.0, 485060.0), (6210074.0, 6215074.0))
    ignition_point = (480060 + 800, 6210074 + 2500, 0)

    env = fire_rs.firemodel.propagation.Environment(area, wind_speed=5, wind_dir=0)
    prop = fire_rs.firemodel.propagation.propagate_from_points(env, [ignition_point],
                                                               until=3 * 3600)

    mw = fire_rs.simulation.morse.MorseWildfire(('localhost', 4000), 'demo_fire')
    mw.set_wildfire_prediction_map(prop.prop_data)

    with mw:
        start_time = datetime.now().timestamp()
        current_time = datetime.now().timestamp()
        elapsed = (current_time - start_time)
        while (elapsed * 100) < prop.until_time:
            current_time = datetime.now().timestamp()
            elapsed = (current_time - start_time)
            at = (elapsed * 100)
            mw.update(at)
