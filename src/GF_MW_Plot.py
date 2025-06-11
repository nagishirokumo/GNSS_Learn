import matplotlib.pyplot as plt
import plotly.graph_objects as go
from plotly.subplots import make_subplots
from collections import defaultdict
from datetime import datetime
import pandas as pd
import random
import matplotlib.colors as mcolors

def parse_gnss_data(file_path):
    """
    从文本文件中读取GNSS观测数据并解析

    参数:
        file_path (str): 数据文件路径

    返回:
        dict: 按卫星PRN号分类的观测数据字典
    """
    data = defaultdict(list)

    with open(file_path, 'r') as file:
        lines = file.readlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if not line:
                i += 1
                continue

            # 解析头行(UTC时间和观测值数量)
            parts = line.split()
            if len(parts) == 7:  # 格式: YYYY MM DD HH MM SS n
                try:
                    year, month, day, hour, minute, second, n_obs = map(int, parts)
                    timestamp = datetime(year, month, day, hour, minute, second)
                    i += 1

                    # 解析接下来的n_obs行观测数据
                    for _ in range(n_obs):
                        if i >= len(lines):
                            break
                        obs_line = lines[i].strip()
                        if obs_line:
                            obs_parts = obs_line.split()
                            if len(obs_parts) >= 4:
                                prn = obs_parts[0]
                                pseudo_gf = float(obs_parts[1])
                                phase_gf = float(obs_parts[2])
                                mw = float(obs_parts[3])

                                data[prn].append({
                                    'timestamp': timestamp,
                                    'pseudo_gf': pseudo_gf,
                                    'phase_gf': phase_gf,
                                    'mw': mw
                                })
                        i += 1
                except (ValueError, IndexError) as e:
                    print(f"解析错误在行 {i+1}: {e}")
                    i += 1
            else:
                i += 1
    return data

def parse_gnss_data_df(file_path):
    """
    从文本文件中读取GNSS观测数据并解析为DataFrame

    参数:
        file_path (str): 数据文件路径

    返回:
        pd.DataFrame: 包含所有观测数据的DataFrame
    """
    data = []

    with open(file_path, 'r') as file:
        lines = file.readlines()
        i = 0
        while i < len(lines):
            line = lines[i].strip()
            if not line:
                i += 1
                continue

            # 解析头行(UTC时间和观测值数量)
            parts = line.split()
            if len(parts) == 7:  # 格式: YYYY MM DD HH MM SS n
                try:
                    year, month, day, hour, minute, second, n_obs = map(int, parts)
                    timestamp = datetime(year, month, day, hour, minute, second)
                    i += 1

                    # 解析接下来的n_obs行观测数据
                    for _ in range(n_obs):
                        if i >= len(lines):
                            break
                        obs_line = lines[i].strip()
                        if obs_line:
                            obs_parts = obs_line.split()
                            if len(obs_parts) >= 4:
                                prn = obs_parts[0]
                                system = prn[0]  # 卫星系统(G, C, E等)
                                pseudo_gf = float(obs_parts[1])
                                phase_gf = float(obs_parts[2])
                                mw = float(obs_parts[3])

                                data.append({
                                    'timestamp': timestamp,
                                    'PRN': prn,
                                    'System': system,
                                    'Pseudorange_GF': pseudo_gf,
                                    'Phase_GF': phase_gf,
                                    'MW': mw
                                })
                        i += 1
                except (ValueError, IndexError) as e:
                    print(f"解析错误在行 {i+1}: {e}")
                    i += 1
            else:
                i += 1

    return pd.DataFrame(data)

def plot_gnss_observations(data, output_file=None):
    """
    绘制GNSS观测值时间序列图

    参数:
        data (dict): 解析后的观测数据
        output_file (str, optional): 保存图像的文件路径
    """
    if not data:
        print("没有有效数据可绘制")
        return

    # 创建3个子图
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 16))
    fig.suptitle('GNSS Observation Values Time Series', fontsize=14)

    # 获取卫星PRN并排序
    prns = sorted(data.keys(), key=lambda x: (x[0], int(x[1:])))

    # 生成随机颜色列表
    colors = list(mcolors.TABLEAU_COLORS.values())  # 使用Tableau调色板
    random.shuffle(colors)  # 打乱颜色顺序
    color_dict = {prn: colors[i % len(colors)] for i, prn in enumerate(prns)}

    # 绘制每个卫星的数据
    for prn in prns:
        color = color_dict[prn]

        timestamps = [obs['timestamp'] for obs in data[prn]]
        pseudo_gf = [obs['pseudo_gf'] for obs in data[prn]]
        phase_gf = [obs['phase_gf'] for obs in data[prn]]
        mw = [obs['mw'] for obs in data[prn]]

        # 伪距GF
        ax1.plot(timestamps, pseudo_gf, 'o-', color=color, label=prn, markersize=4)

        # 载波相位GF
        ax2.plot(timestamps, phase_gf, 'o-', color=color, label=prn, markersize=4)

        # MW组合
        ax3.plot(timestamps, mw, 'o-', color=color, label=prn, markersize=4)

    # 设置子图属性
    ax1.set_ylabel('Pseudorange GF (m)', fontsize=10)
    ax1.set_title('Pseudorange Geometry-Free Combination', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.tick_params(axis='x', rotation=45)

    ax2.set_ylabel('Phase GF (m)', fontsize=10)
    ax2.set_title('Carrier Phase Geometry-Free Combination', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.tick_params(axis='x', rotation=45)

    ax3.set_ylabel('MW (m)', fontsize=10)
    ax3.set_title('Melbourne-Wübbena Combination', fontsize=12)
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.tick_params(axis='x', rotation=45)

    # 添加图例
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc='right', bbox_to_anchor=(1.15, 0.5),
               title='Satellite PRN', fontsize=8)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, bbox_inches='tight', dpi=300)
        print(f"图形已保存到 {output_file}")
    else:
        plt.show()

def create_interactive_gnss_plot(df, output_html="gnss_plot.html"):
    """
    创建交互式GNSS观测值时间序列图

    参数:
        df (pd.DataFrame): 包含观测数据的DataFrame
        output_html (str): 输出的HTML文件路径
    """
    if df.empty:
        print("没有有效数据可绘制")
        return

    # 创建包含3个子图的图形
    fig = make_subplots(rows=3, cols=1,
                        subplot_titles=(
                            "Pseudorange Geometry-Free Combination",
                            "Carrier Phase Geometry-Free Combination",
                            "Melbourne-Wübbena Combination"
                        ),
                        vertical_spacing=0.1)

    # 获取唯一的PRN列表并按系统排序
    prns = sorted(df['PRN'].unique(), key=lambda x: (x[0], int(x[1:])))

    # 生成随机颜色列表
    colors = ['rgb'+str(tuple(random.randint(0, 255) for _ in range(3))) for _ in prns]

    # 为每个PRN添加轨迹
    for i, prn in enumerate(prns):
        color = colors[i]
        prn_df = df[df['PRN'] == prn]

        # 伪距GF
        fig.add_trace(
            go.Scatter(
                x=prn_df['timestamp'],
                y=prn_df['Pseudorange_GF'],
                name=prn,
                mode='lines+markers',
                marker=dict(size=5, color=color),
                line=dict(width=1, color=color),
                legendgroup=prn,
                showlegend=True,
                hoverinfo='x+y+name',
                hovertemplate='%{x|%Y-%m-%d %H:%M:%S}<br>%{y:.4f} m'
            ),
            row=1, col=1
        )

        # 载波相位GF
        fig.add_trace(
            go.Scatter(
                x=prn_df['timestamp'],
                y=prn_df['Phase_GF'],
                name=prn,
                mode='lines+markers',
                marker=dict(size=5, color=color),
                line=dict(width=1, color=color),
                legendgroup=prn,
                showlegend=False,  # 只在第一个子图显示图例
                hoverinfo='x+y+name',
                hovertemplate='%{x|%Y-%m-%d %H:%M:%S}<br>%{y:.4f} m'
            ),
            row=2, col=1
        )

        # MW组合
        fig.add_trace(
            go.Scatter(
                x=prn_df['timestamp'],
                y=prn_df['MW'],
                name=prn,
                mode='lines+markers',
                marker=dict(size=5, color=color),
                line=dict(width=1, color=color),
                legendgroup=prn,
                showlegend=False,
                hoverinfo='x+y+name',
                hovertemplate='%{x|%Y-%m-%d %H:%M:%S}<br>%{y:.4f} m'
            ),
            row=3, col=1
        )

    # 更新布局
    fig.update_layout(
        title_text='GNSS Observation Values Time Series',
        height=900,
        hovermode='x unified',
        legend=dict(
            title='Satellite PRN',
            itemsizing='constant',
            tracegroupgap=5
        )
    )

    # 更新y轴标签
    fig.update_yaxes(title_text="Pseudorange GF (m)", row=1, col=1)
    fig.update_yaxes(title_text="Phase GF (m)", row=2, col=1)
    fig.update_yaxes(title_text="MW (m)", row=3, col=1)

    # 添加时间轴同步
    fig.update_xaxes(matches='x')

    # 保存为交互式HTML文件
    fig.write_html(output_html, include_plotlyjs='cdn', full_html=True)
    print(f"交互式图形已保存到 {output_html}")

    return fig

def plot_gnss_observations_dif(data, output_file=None):
    """
    绘制GNSS观测值时间序列图

    参数:
        data (dict): 解析后的观测数据
        output_file (str, optional): 保存图像的文件路径
    """
    if not data:
        print("没有有效数据可绘制")
        return

    # 创建3个子图
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 16))
    fig.suptitle('GNSS Observation Values Time Series', fontsize=14)

    # 获取卫星PRN并排序
    prns = sorted(data.keys(), key=lambda x: (x[0], int(x[1:])))

    # 生成随机颜色列表
    colors = list(mcolors.TABLEAU_COLORS.values())  # 使用Tableau调色板
    random.shuffle(colors)  # 打乱颜色顺序
    color_dict = {prn: colors[i % len(colors)] for i, prn in enumerate(prns)}

    # 绘制每个卫星的数据
    for prn in prns:
        color = color_dict[prn]

        timestamps = [obs['timestamp'] for obs in data[prn]]
        pseudo_gf = [obs['pseudo_gf'] for obs in data[prn]]
        phase_gf = [obs['phase_gf'] for obs in data[prn]]
        mw = [obs['mw'] for obs in data[prn]]

        # 伪距GF
        ax1.plot(timestamps, pseudo_gf, 'o-', color=color, label=prn, markersize=4)

        # 载波相位GF
        ax2.plot(timestamps, phase_gf, 'o-', color=color, label=prn, markersize=4)

        # MW组合
        ax3.plot(timestamps, mw, 'o-', color=color, label=prn, markersize=4)

    # 设置子图属性
    ax1.set_ylabel('Pseudorange GF (m)', fontsize=10)
    ax1.set_title('Pseudorange Geometry-Free Combination', fontsize=12)
    ax1.grid(True, linestyle='--', alpha=0.6)
    ax1.tick_params(axis='x', rotation=45)

    ax2.set_ylabel('Phase GF (m)', fontsize=10)
    ax2.set_title('Carrier Phase Geometry-Free Combination', fontsize=12)
    ax2.grid(True, linestyle='--', alpha=0.6)
    ax2.tick_params(axis='x', rotation=45)

    ax3.set_ylabel('MW (m)', fontsize=10)
    ax3.set_title('Melbourne-Wübbena Combination', fontsize=12)
    ax3.grid(True, linestyle='--', alpha=0.6)
    ax3.tick_params(axis='x', rotation=45)

    # 添加图例
    handles, labels = ax1.get_legend_handles_labels()
    fig.legend(handles, labels, loc='right', bbox_to_anchor=(1.15, 0.5),
               title='Satellite PRN', fontsize=8)

    plt.tight_layout()

    if output_file:
        plt.savefig(output_file, bbox_inches='tight', dpi=300)
        print(f"图形已保存到 {output_file}")
    else:
        plt.show()

def create_interactive_gnss_dif_plot(df, output_html="gnss_plot.html"):
    """
    创建交互式GNSS观测值时间序列图

    参数:
        df (pd.DataFrame): 包含观测数据的DataFrame
        output_html (str): 输出的HTML文件路径
    """
    if df.empty:
        print("没有有效数据可绘制")
        return

    # 创建包含3个子图的图形
    fig = make_subplots(rows=3, cols=1,
                        subplot_titles=(
                            "Pseudorange Geometry-Free Combination",
                            "Carrier Phase Geometry-Free Combination",
                            "Melbourne-Wübbena Combination"
                        ),
                        vertical_spacing=0.1)

    # 获取唯一的PRN列表并按系统排序
    prns = sorted(df['PRN'].unique(), key=lambda x: (x[0], int(x[1:])))

    # 生成随机颜色列表
    colors = ['rgb'+str(tuple(random.randint(0, 255) for _ in range(3))) for _ in prns]

    # 为每个PRN添加轨迹
    for i, prn in enumerate(prns):
        color = colors[i]
        prn_df = df[df['PRN'] == prn]

        # 伪距GF
        fig.add_trace(
            go.Scatter(
                x=prn_df['timestamp'],
                y=prn_df['Pseudorange_GF'],
                name=prn,
                mode='lines+markers',
                marker=dict(size=5, color=color),
                line=dict(width=1, color=color),
                legendgroup=prn,
                showlegend=True,
                hoverinfo='x+y+name',
                hovertemplate='%{x|%Y-%m-%d %H:%M:%S}<br>%{y:.4f} m'
            ),
            row=1, col=1
        )

        # 载波相位GF
        fig.add_trace(
            go.Scatter(
                x=prn_df['timestamp'],
                y=prn_df['Phase_GF'],
                name=prn,
                mode='lines+markers',
                marker=dict(size=5, color=color),
                line=dict(width=1, color=color),
                legendgroup=prn,
                showlegend=False,  # 只在第一个子图显示图例
                hoverinfo='x+y+name',
                hovertemplate='%{x|%Y-%m-%d %H:%M:%S}<br>%{y:.4f} m'
            ),
            row=2, col=1
        )

        # MW组合
        fig.add_trace(
            go.Scatter(
                x=prn_df['timestamp'],
                y=prn_df['MW'],
                name=prn,
                mode='lines+markers',
                marker=dict(size=5, color=color),
                line=dict(width=1, color=color),
                legendgroup=prn,
                showlegend=False,
                hoverinfo='x+y+name',
                hovertemplate='%{x|%Y-%m-%d %H:%M:%S}<br>%{y:.4f} m'
            ),
            row=3, col=1
        )

    # 更新布局
    fig.update_layout(
        title_text='GNSS Observation Values Time Series',
        height=900,
        hovermode='x unified',
        legend=dict(
            title='Satellite PRN',
            itemsizing='constant',
            tracegroupgap=5
        )
    )

    # 更新y轴标签
    fig.update_yaxes(title_text="Pseudorange GF (m)", row=1, col=1)
    fig.update_yaxes(title_text="Phase GF (m)", row=2, col=1)
    fig.update_yaxes(title_text="MW (m)", row=3, col=1)

    # 添加时间轴同步
    fig.update_xaxes(matches='x')

    # 保存为交互式HTML文件
    fig.write_html(output_html, include_plotlyjs='cdn', full_html=True)
    print(f"交互式图形已保存到 {output_html}")

    return fig

# 使用示例
if __name__ == "__main__":
    # 输入文件路径
    input_file = "RINEX_Obs_GF&WM.txt"  # 替换为你的文件路径
    input_file_dif="RINEX_Obs_GF&WM_Dif.txt"
    output_image = "GF&WM_time_series.png"  # 输出图像路径(可选)
    output_html = "interactive_gnss_plot.html"  # 输出HTML文件路径
    output_image_dif = "GF&WM_dif_time_series.png"  # 输出图像路径(可选)
    output_html_dif = "interactive_gnss_plot_dif.html"  # 输出HTML文件路径
    # 解析数据
    gnss_data = parse_gnss_data(input_file)

    gnss_df = parse_gnss_data_df(input_file)

    gnss_data_dif = parse_gnss_data(input_file_dif)

    gnss_df_dif = parse_gnss_data_df(input_file_dif)
    if gnss_data:
        print(f"成功解析 {len(gnss_data)} 颗卫星的数据")
        # 绘制图形
        plot_gnss_observations(gnss_data, output_image)
        # 创建交互式图形
        fig = create_interactive_gnss_plot(gnss_df, output_html_dif)
        # 绘制图形
        plot_gnss_observations_dif(gnss_data_dif, output_image_dif)
        # 创建交互式图形
        fig = create_interactive_gnss_dif_plot(gnss_df_dif, output_html_dif)
    else:
        print("未能解析有效数据")