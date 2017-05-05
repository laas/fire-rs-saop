import random
import numpy as np
import scipy.spatial
import scipy.spatial.distance
import fire_rs.planning.uav as uav
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque, defaultdict
from collections import namedtuple
from sklearn.cluster import DBSCAN
from sklearn import metrics
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler
from sklearn import datasets
from scipy.stats import linregress
import logging
from typing import Sequence


camera_fov = (10, 10) # Horizontal, Vertical

# Generate sample data
centers = [[1, 1], [-1, -1], [1, -1], [2, 2]]
points_of_interest, labels_true = make_blobs(n_samples=200, cluster_std=1, centers=20,
                            random_state=22)
points_of_interest = points_of_interest * 10


def density_weight(all_nodes):
    '''Calculate a relevance weight of each node.

    For every node, its weight is the sum of the inverse of the squared eculidean distance to any other node.
    '''
    # TODO: the distance to the UAV should be taken in account, so the closest cumulus of points
    # get more importance than those that are further.
    distance_matrix = scipy.spatial.distance.cdist(all_nodes, all_nodes, 'euclidean')
    for i in range(len(distance_matrix)):
        distance_matrix[i, i] = np.inf

    weights = np.sum(1 / distance_matrix ** 2, axis=0)
    max_weight = np.max(weights)
    return weights / max_weight


def tesselate():
    poi_weights = density_weight(points_of_interest)

    tesselation = scipy.spatial.Delaunay(points_of_interest)

    fig = plt.figure()

    plt.triplot(points_of_interest[:, 0], points_of_interest[:, 1], tesselation.simplices.copy())
    plt.scatter(points_of_interest[:, 0], points_of_interest[:, 1], poi_weights * 100, c='red')

    plt.show()


#######################


class VectorLine(namedtuple('VectorLine', ('point', 'vector'))):
    __slots__ = ()

    def to_point_slope_form(self):
        return PointSlopeLine(self.point, self.vector[1]/self.vector[0])

    def evaluate(self, mu):
        return self.point + mu * self.vector


class PointSlopeLine(namedtuple('PointSlopeLine',('point', 'slope'))):
    __slots__ = ()

    def to_vector_form(self):
        line_angle = np.arctan(self.slope)
        n = np.array([np.cos(line_angle), np.sin(line_angle)])
        return VectorLine(self.point, n)

    def evaluate(self, x):
        return self.slope * (x - self.point[0]) + self.point[1]


def cluster(point_cloud, epsilon=10, min_samples=2, metric='euclidean', plot_clusters=True):
    """Cluster a point cloud.
    
    Returns:
        clustered: list of clusters
        unclustered: The rest of points that do not belong to any cluster
        """
    db = DBSCAN(eps=epsilon, min_samples=min_samples, metric=metric).fit(point_cloud)
    core_samples_mask = np.zeros_like(db.labels_, dtype=bool)
    core_samples_mask[db.core_sample_indices_] = True
    labels = db.labels_

    # Number of clusters in labels, ignoring noise if present.
    n_clusters_ = len(set(labels)) - (1 if -1 in labels else 0)
    unique_labels = set(labels)

    if plot_clusters:
        # Plot result
        # Black removed and is used for noise instead.
        colors = plt.cm.Spectral(np.linspace(0, 1, len(unique_labels)))
        for k, col in zip(unique_labels, colors):
            if k == -1:
                # Black used for noise.
                col = 'k'

            class_member_mask = (labels == k)

            xy = point_cloud[class_member_mask & core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=col,
                     markeredgecolor='k', markersize=10)

            xy = point_cloud[class_member_mask & ~core_samples_mask]
            plt.plot(xy[:, 0], xy[:, 1], 'o', markerfacecolor=col,
                     markeredgecolor='k', markersize=5)

        plt.title('Estimated number of clusters: {}'.format(n_clusters_))
        plt.show()

    clustered = []
    unclustered = []
    for c in unique_labels:
        if c != -1:
            clustered.append(point_cloud[labels==c])
        else:
            unclustered = point_cloud[labels==c]

    return clustered, unclustered


def intersection_of_lines(*lines: 'Sequence[VectorLine]'):
    assert len(lines) == 2

    # solving linear equation system using cramer's rule
    M = np.array([[lines[0].vector[0], -lines[1].vector[0]],[lines[0].vector[1],-lines[1].vector[1]]])
    C = lines[1].point - lines[0].point

    x = np.linalg.det(np.column_stack([C, M[..., 1]])) / np.linalg.det(M)
    y = np.linalg.det(np.column_stack([M[..., 0], C])) / np.linalg.det(M)

    return np.array([x, y])


def distance_point_to_line(point, line: 'VectorLine'):
    """Calculate the distance between a point and a line.

    https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line#Vector_formulation
    """
    distance = np.linalg.norm((line.point - point) - ((line.point - point).dot(line.vector)) * line.vector)
    return distance


class SegmentObservationPath():

    def __init__(self, points, camera_fov):
        self.points = points
        self.camera_fov = camera_fov
        self.observed = np.zeros(self.points.shape[0], dtype=np.bool)
        self.line = None
        self.regression = None
        self.segment = None

        self._linear_regression()
        self._determine_coverage()
        self._determine_segment()


    def _linear_regression(self):
        """Get the line that fits the best a set of points using linear regression"""
        # TODO: Improve using a robust regression estimator (ie. RANSAC)
        self.regression = linregress(self.points[..., 0], self.points[..., 1])
        logging.info("Cluster: {}\n\tslope: {}\n\ty(0): {}\n\tr: {}\n\tp: {}\n\tstderr: {}".format(
            self.points, self.regression.slope, self.regression.intercept, self.regression.rvalue, self.regression.pvalue, self.regression.stderr))
        self.line = PointSlopeLine(np.array([0, self.regression.intercept]), self.regression.slope).to_vector_form()

    def _determine_coverage(self):
        """Evaluate how many points are observed flying over a linear trajectory"""
        for i, point in enumerate(self.points):
            if distance_point_to_line(point, self.line) < self.camera_fov[0] / 2:
                self.observed[i] = True
            else:
                self.observed[i] = False

        logging.info("{}/{} points observed".format(np.count_nonzero(self.observed), self.points.shape[0]))

    def _determine_segment(self):
        line_pslope = self.line.to_point_slope_form()

        most_left_point = min(self.points, key=lambda p: p[0])
        p0 = np.array([most_left_point[0], line_pslope.evaluate(most_left_point[0])])

        most_right_point = max(self.points, key=lambda p: p[0])
        p1 = np.array([most_right_point[0], line_pslope.evaluate(most_right_point[0])])

        self.segment = np.array([p0, p1])


def linear_paths():
    clusters, unclustered = cluster(points_of_interest)
    for c in clusters:
        obs_path = SegmentObservationPath(c, camera_fov)
        observed = c[obs_path.observed]
        plt.plot(observed[:, 0], observed[:, 1], 'o', markerfacecolor='green',
                 markeredgecolor='green', markersize=10)
        plt.plot([obs_path.segment[0][0], obs_path.segment[1][0]], [obs_path.segment[0][1], obs_path.segment[1][1]], linewidth=3, color='k')
        # pl1 = obs_path._left_perp_line.point + obs_path._left_perp_line.vector*-50
        # pl2 = obs_path._left_perp_line.point + obs_path._left_perp_line.vector*50
        # plt.plot([pl1[0], pl2[0]], [pl1[1], pl2[1]])


if __name__ == '__main__':
    # tesselate()
    logging.basicConfig(level=logging.DEBUG)
    linear_paths()
    print("stop")

