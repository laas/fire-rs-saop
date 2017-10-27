# Copyright (c) 2017, CNRS-LAAS
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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
from sklearn.linear_model import LinearRegression, RANSACRegressor
from sklearn.datasets.samples_generator import make_blobs
from sklearn.preprocessing import StandardScaler
from sklearn import datasets
from scipy.stats import linregress
import logging
from typing import Sequence


class VectorLine(namedtuple('VectorLine', ('point', 'vector'))):
    __slots__ = ()

    def to_point_slope_form(self):
        return PointSlopeLine(self.point, self.vector[1] / self.vector[0])

    def evaluate(self, mu):
        return self.point + mu * self.vector


class PointSlopeLine(namedtuple('PointSlopeLine', ('point', 'slope'))):
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
            clustered.append(point_cloud[labels == c])
        else:
            unclustered = point_cloud[labels == c]

    return clustered, unclustered


def intersection_of_lines(*lines: 'Sequence[VectorLine]'):
    assert len(lines) == 2

    # solving linear equation system using cramer's rule
    M = np.array([[lines[0].vector[0], -lines[1].vector[0]], [lines[0].vector[1], -lines[1].vector[1]]])
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
    def __init__(self, points, camera_fov, regressor=LinearRegression()):
        self.points = points
        self.camera_fov = camera_fov
        self.regressor = regressor

        self.observed = np.zeros(self.points.shape[0], dtype=np.bool)
        self.line = None
        self.regression = None
        self.segment = None

        self._regression()
        self._observe()
        self._determine_segment()

    @property
    def coverage(self):
        return np.count_nonzero(self.observed) / self.points.shape[0]

    def _regression(self):
        """Get the line that fits the best a set of points using linear regression"""
        self.regressor.fit(X=self.points[..., 0][..., np.newaxis], y=self.points[..., 1])
        if isinstance(self.regressor, RANSACRegressor):
            logging.info("Cluster: {}\n\tslope: {}\n\ty(0): {}\n\tresidues: {}".format(
                self.points, self.regressor.estimator_.coef_, self.regressor.estimator_.intercept_,
                self.regressor.estimator_.residues_))
            self.line = PointSlopeLine(np.array([0, self.regressor.estimator_.intercept_]),
                                       float(self.regressor.estimator_.coef_)).to_vector_form()
        else:
            logging.info("Cluster: {}\n\tslope: {}\n\ty(0): {}\n\tresidues: {}".format(
                self.points, self.regressor.coef_, self.regressor.intercept_, self.regressor.residues_))
            self.line = PointSlopeLine(np.array([0, self.regressor.intercept_]),
                                       float(self.regressor.coef_)).to_vector_form()

    def _observe(self):
        """Evaluate how many points are observed flying over a linear trajectory"""
        for i, point in enumerate(self.points):
            if distance_point_to_line(point, self.line) < self.camera_fov[0] / 2:
                self.observed[i] = True
            else:
                self.observed[i] = False

        logging.info("{}: {}/{} points observed".format(str(type(self.regressor)), np.count_nonzero(self.observed),
                                                        self.points.shape[0]))

    def _determine_segment(self):
        line_pslope = self.line.to_point_slope_form()

        most_left_point = min(self.points[self.observed], key=lambda p: p[0])
        p0 = np.array([most_left_point[0], line_pslope.evaluate(most_left_point[0])])

        most_right_point = max(self.points[self.observed], key=lambda p: p[0])
        p1 = np.array([most_right_point[0], line_pslope.evaluate(most_right_point[0])])

        self.segment = np.array([p0, p1])


def process_points(points, camera_fov, eps=10):
    clusters, unclustered = cluster(points, epsilon=eps)
    n_observations_cumulated = {'LinearRegression': 0, 'RANSACRegressor': 0}
    for c in clusters:
        obs_paths = []
        to_observe = c.copy()
        while to_observe.shape[0] > max(1, int(0.01 * c.shape[0])):
            path = SegmentObservationPath(to_observe, camera_fov, regressor=RANSACRegressor())
            obs_paths.append(path)
            to_observe = to_observe[~path.observed]

            plt.plot(path.points[path.observed][:, 0], path.points[path.observed][:, 1], 'o', markerfacecolor='blue',
                     markeredgecolor='blue', markersize=5)
            plt.plot([path.segment[0][0], path.segment[1][0]],
                     [path.segment[0][1], path.segment[1][1]],
                     linewidth=3, color='blue')
            plt.show()

    print(n_observations_cumulated)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG)
    camera_fov = (10, 10)  # Horizontal, Vertical

    points_of_interest, labels_true = make_blobs(n_samples=200, cluster_std=1, centers=20,
                                                 random_state=21)
    points_of_interest = points_of_interest * 10

    process_points(points_of_interest, camera_fov)
    print("stop")
