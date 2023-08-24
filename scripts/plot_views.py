import argparse
import pathlib

import h5py
import matplotlib.pyplot as plt
import numpy as np


def open_h5_file(fpath: pathlib.Path) -> h5py.Group:
    """
    Opens an HDF5 file. 
    """
    h5_file  = h5py.File(fpath, "r")
    return h5_file


def get_policy_group(h5_file: pathlib.Path) -> h5py.Group:
    """
    Retrieves the Policy group within an HDF5 file. 
    """
    h5_group = h5_file["Policy"]
    return h5_group


def get_reconstruction_group(h5_file: h5py.File) -> h5py.Group:
    """
    Retrieves the Reconstruction group within an HDF5 file. 
    """
    h5_group = h5_file["Reconstruction"]
    return h5_group


def plot_voxel_grid(reconstruction_group: h5py.Group,  ax: plt.Axes) -> None:
    """
    Plots a wireframe of the voxel grid with the lower corner, upper corner, and center marked. 
    """
    upper = reconstruction_group.attrs["VoxelGrid Dimensions"]
    lower = np.zeros_like(upper)
    center = (upper - lower) / 2

    side0 = np.array([[lower[0], lower[1], lower[2]],
                      [upper[0], lower[1], lower[2]],
                      [upper[0], upper[1], lower[2]],
                      [lower[0], upper[1], lower[2]],
                      [lower[0], lower[1], lower[2]]
                    ])
    side1 = np.array([[lower[0], lower[1], lower[2]],
                      [lower[0], upper[1], lower[2]],
                      [lower[0], upper[1], upper[2]],
                      [lower[0], lower[1], upper[2]],
                      [lower[0], lower[1], lower[2]]
                    ])
    side2 = np.array([[lower[0], lower[1], upper[2]],
                      [upper[0], lower[1], upper[2]],
                      [upper[0], upper[1], upper[2]],
                      [lower[0], upper[1], upper[2]],
                      [lower[0], lower[1], upper[2]]
                    ])
    side3 = np.array([[upper[0], lower[1], lower[2]],
                      [upper[0], upper[1], lower[2]],
                      [upper[0], upper[1], upper[2]],
                      [upper[0], lower[1], upper[2]],
                      [upper[0], lower[1], lower[2]]
                    ])
    
    ax.scatter(lower[0],  lower[1],  lower[2],  color='C0')
    ax.scatter(upper[0],  upper[1],  upper[2],  color='C0')
    ax.scatter(center[0], center[1], center[2], color='C0', marker='x')

    ax.plot_wireframe(side0[:, 0], side0[:, 1], side0[:, 2])
    ax.plot_wireframe(side1[:, 0], side1[:, 1], side1[:, 2])
    ax.plot_wireframe(side2[:, 0], side2[:, 1], side2[:, 2])
    ax.plot_wireframe(side3[:, 0], side3[:, 1], side3[:, 2])


def add_view_to_plot(extr: np.array, view_id: str, ax: plt.Axes, accept: bool,
                     parsed_args: argparse.Namespace = None) -> None:
    """
    Adds a view to the plot. The whole extrinsic matrix is plotted and a view ID is placed as text
    near the location of the view. If the accepted flag is True the origin and text are colored 
    green, but if False these are colored red. 
    """
    green = "green" 
    blue  = "blue" 
    red   = "red" 
    origin_color = green
    if accept is False:
        origin_color = red

    x_axis = extr[0:3, 0]
    y_axis = extr[0:3, 1]
    z_axis = extr[0:3, 2]
    origin = extr[0:3, 3]

    text = x_axis + y_axis + z_axis
    text = 0.1 * (text / np.linalg.norm(text))
    text = origin + text

    if parsed_args and parsed_args.only_z is False:
        ax.quiver(origin[0], origin[1], origin[2], x_axis[0], x_axis[1], x_axis[2], color=red)
        ax.quiver(origin[0], origin[1], origin[2], y_axis[0], y_axis[1], y_axis[2], color=green)
    ax.quiver(origin[0], origin[1], origin[2], z_axis[0], z_axis[1], z_axis[2], color=blue)
    ax.scatter(origin[0], origin[1], origin[2], color=origin_color)
    ax.text(text[0], text[1], text[2], view_id, color=origin_color)


def add_accepted_group_to_plot(group: h5py.Group, ax: plt.Axes,
                               parsed_args: argparse.Namespace = None) -> None:
    for (dset_name, dset) in group.items():
        if isinstance(dset, h5py.Dataset):
            add_view_to_plot(np.array(dset), dset_name, ax, accept=True, parsed_args=parsed_args)


def add_rejected_group_to_plot(group: h5py.Group, ax: plt.Axes, 
                               parsed_args: argparse.Namespace = None) -> None:
    for (dset_name, dset) in group.items():
        if isinstance(dset, h5py.Dataset):
            add_view_to_plot(np.array(dset), dset_name, ax, accept=False, parsed_args=parsed_args)


def plot_views(policy_group: h5py.Group, ax: plt.Axes, parsed_args: argparse.Namespace = None) -> None:
    """
    Creates a plot of the accuracy and precision at each step.
    """
    for (_, group) in policy_group.items():
        if isinstance(group, h5py.Group):
            for (view_group_name, group) in group.items():
                if (view_group_name == "ACCEPT"):
                    add_accepted_group_to_plot(group, ax, parsed_args)
                if (view_group_name == "REJECT") and parsed_args and parsed_args.plot_rejects:
                    add_rejected_group_to_plot(group, ax, parsed_args)


def main(parsed_args: argparse.Namespace) -> None:
    """
    Program entry point.
    """
    h5_file = open_h5_file(parsed_args.file)

    fig = plt.figure()
    ax: plt.Axes = fig.add_subplot(111, projection='3d')
    ax.set_title("Reconstruction Views")
    ax.set_xlim((-5, 5))
    ax.set_ylim((-5, 5))
    ax.set_zlim((-5, 5))

    reconstruction_group = get_reconstruction_group(h5_file)
    plot_voxel_grid(reconstruction_group, ax)

    policy_group = get_policy_group(h5_file)
    plot_views(policy_group, ax, parsed_args)

    plt.show()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        prog='Plot Views',
        description='Reads a ForgeScan HDF5 file and plots the confusion matrix data, if such'
                    'Data is present.'
    )
    parser.add_argument(
        "-f", "--file",
        type=pathlib.Path,
        required=True,
        help="Path to the HDF5 file to load."
    )
    parser.add_argument(
        "--plot-rejects",
        action="store_true",
        default=False,
        help="Plots the rejected views too."
    )
    parser.add_argument(
        "--only-z",
        action="store_true",
        default=False,
        help="Will only plot the Z-axis of the view."
    )
    args = parser.parse_args()

    main(args)
