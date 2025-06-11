import pandas as pd
import numpy as np
import chardet
import plotly.graph_objs as go
from scipy.interpolate import interp1d
import matplotlib.pyplot as plt


def load_broadcast_csv(csv_file):
    with open(csv_file, 'rb') as f:
        result = chardet.detect(f.read())
    encoding = result['encoding']
    print(f"检测到广播星历编码: {encoding}")

    df = pd.read_csv(csv_file, encoding=encoding)
    return df


def parse_sp3(file_path):
    satellites = {}
    times = []

    with open(file_path, 'r') as f:
        for line in f:
            if line.startswith('*'):
                year = int(line[3:7])
                month = int(line[8:10])
                day = int(line[11:13])
                hour = int(line[14:16])
                minute = int(line[17:19])
                second = float(line[20:31])
                times.append((year, month, day, hour, minute, second))
            elif line.startswith('P'):
                sat_id = line[1:4].strip()
                x = float(line[4:18]) * 1000  # km -> m
                y = float(line[18:32]) * 1000
                z = float(line[32:46]) * 1000
                clock = float(line[46:60])
                if sat_id not in satellites:
                    satellites[sat_id] = []
                satellites[sat_id].append([x, y, z, clock])
    return np.array(times), satellites


def earth_fixed_to_inertial(x, y, z, delta_t):
    omega = 7.2921150e-5  # 地球自转角速度 (rad/s)
    theta = omega * delta_t

    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    x_inertial = cos_theta * x - sin_theta * y
    y_inertial = sin_theta * x + cos_theta * y
    z_inertial = z  # Z轴不变

    return x_inertial, y_inertial, z_inertial


def interpolate_orbit_segments(times, orbit_data, max_gap_seconds=900, interp_interval=60):
    times_sec = np.array([t[3]*3600 + t[4]*60 + t[5] for t in times])
    t0 = times_sec[0]


    inertial_orbit = []
    for t, p in zip(times_sec, orbit_data):
        delta_t = t - t0
        x_inertial, y_inertial, z_inertial = earth_fixed_to_inertial(p[0], p[1], p[2], delta_t)
        inertial_orbit.append([t, x_inertial, y_inertial, z_inertial])

    inertial_orbit = np.array(inertial_orbit)


    segments = []
    current_segment = [inertial_orbit[0]]
    for i in range(1, len(inertial_orbit)):
        if inertial_orbit[i, 0] - inertial_orbit[i-1, 0] > max_gap_seconds:
            segments.append(np.array(current_segment))
            current_segment = []
        current_segment.append(inertial_orbit[i])
    if current_segment:
        segments.append(np.array(current_segment))


    interpolated_segments = []
    for seg in segments:
        if len(seg) < 2:
            continue
        t_seg = seg[:, 0]
        x_seg = seg[:, 1]
        y_seg = seg[:, 2]
        z_seg = seg[:, 3]

        interp_time = np.arange(t_seg[0], t_seg[-1], interp_interval)
        fx = interp1d(t_seg, x_seg, kind='cubic')
        fy = interp1d(t_seg, y_seg, kind='cubic')
        fz = interp1d(t_seg, z_seg, kind='cubic')

        x_interp = fx(interp_time)
        y_interp = fy(interp_time)
        z_interp = fz(interp_time)

        interpolated_segments.append((x_interp, y_interp, z_interp))

    return interpolated_segments


def generate_color_list(num_colors):
    cmap = plt.get_cmap('tab20')  # 20种颜色
    colors = [cmap(i % 20) for i in range(num_colors)]
    rgb_colors = [f'rgb({int(r*255)}, {int(g*255)}, {int(b*255)})' for r, g, b, _ in colors]
    return rgb_colors


def plot_orbits(broadcast_df, sp3_times, sp3_sats, system_prefix='C', output_html_path=None):
    fig = go.Figure()


    prn_list = sorted(broadcast_df['PRN号'].unique())
    colors = generate_color_list(len(prn_list))
    prn_color_map = {prn: color for prn, color in zip(prn_list, colors)}


    for prn, group in broadcast_df.groupby('PRN号'):
        if isinstance(prn, str) and not prn.startswith(system_prefix):
            continue

        color = prn_color_map.get(prn, 'blue')

        hover_text = [
            f'广播 | PRN: {prn}<br>周内秒: {tow:.2f}' for tow in group['周内秒']
        ]

        fig.add_trace(go.Scatter3d(
            x=group['X坐标'],
            y=group['Y坐标'],
            z=group['Z坐标'],
            mode='lines',
            name=f'广播 {prn}',
            line=dict(width=3, color=color),
            text=hover_text,
            hoverinfo='text'
        ))


    for sat_id, orbit in sp3_sats.items():
        if not sat_id.startswith(system_prefix):
            continue

        prn = sat_id
        color = prn_color_map.get(prn, 'red')

        segments = interpolate_orbit_segments(sp3_times, orbit)

        for idx, (x, y, z) in enumerate(segments):
            fig.add_trace(go.Scatter3d(
                x=x,
                y=y,
                z=z,
                mode='lines',
                name=f'精密 {sat_id}' if idx == 0 else f'精密 {sat_id} (段{idx+1})',
                line=dict(width=8, color=color, dash='dash'),   # <--- 线宽调粗、实线
                hoverinfo='none'
            ))


    fig.update_layout(
        title=f"广播星历 vs 精密星历 卫星轨迹对比 ({system_prefix}系统)",
        scene=dict(
            xaxis_title="X (m)",
            yaxis_title="Y (m)",
            zaxis_title="Z (m)"
        ),
        legend_title="轨迹类型",
        width=1200,
        height=900
    )

    if output_html_path:
        fig.write_html(output_html_path)
        print(f"已保存为: {output_html_path}")
    else:
        fig.show()


if __name__ == '__main__':

    csv_file = '../resource/RINEX_GPS_BDS_Step60.csv'
    sp3_file = '../resource/WUM0MGXFIN_20193350000_01D_15M_ORB.SP3'


    broadcast_df = load_broadcast_csv(csv_file)
    sp3_times, sp3_sats = parse_sp3(sp3_file)


    plot_orbits(broadcast_df, sp3_times, sp3_sats, system_prefix='C', output_html_path='../HTML/BDS轨迹.html')
    plot_orbits(broadcast_df, sp3_times, sp3_sats, system_prefix='G', output_html_path='../HTML/GPS轨迹.html')



