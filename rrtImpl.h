//

//

#ifndef PROJECT_RRTIMPL_H
#define PROJECT_RRTIMPL_H

#endif

#pragma once

#include <iostream>
#include <geometry_msgs/Point.h>
#include <eigen3/Eigen/Dense>
#include <random>

struct Node {
public:
    Node() {}

    Node(geometry_msgs::Point p) : point(p) {}

    int id;
    geometry_msgs::Point point;
    std::vector<Node> children;
    int parentId;
};

class RRT {
public:
    RRT(Node init, Node goal, float sigma, int x_max, int x_min, int y_max, int y_min) : _init(init), _goal(goal),
                                                                                         sigma(sigma), x_max(x_max),
                                                                                         x_min(x_min), y_max(y_max),
                                                                                         y_min(y_min) {
        nodesList.reserve(1000);
        nodesList.push_back(init);
    }

    geometry_msgs::Point getRandomConfig() {
        geometry_msgs::Point point;

        std::random_device rand_dev;
        std::mt19937 generator(rand_dev());
        std::uniform_int_distribution<int> distr(x_min, x_max);

        point.x = distr(generator);
        point.y = distr(generator);
        //todo: check for collisions.
        return point;
    }

    std::map<float, Node> distance_map;

    Node getNearestNode(geometry_msgs::Point p) {
        int n_nodes = nodesList.size();
        if (n_nodes == 1) {
            return (nodesList[0]);
        }
        distance_map.clear();

        for (int i = 0; i < n_nodes; i++) {
            Node treeNode = nodesList[i];
            float d = getEuclideanDistance(p, treeNode.point);
            distance_map[d] = treeNode;
        }

        return distance_map.begin()->second;
    }

    /**
     *
     * @param p1
     * @param p2
     * @return euclidean distance between p1 and p2
     */
    float getEuclideanDistance(geometry_msgs::Point p1, geometry_msgs::Point p2) {
        return std::sqrt(std::pow((p1.x - p2.x), 2) + std::pow((p1.y - p2.y), 2));
    }

    /**
     *
     * @param p1 nearest point
     * @param p2 random config
     * @return point P which is in sigma distance to p1 in the direction of p2
     */
    Node expand(Node p1, Node p2, std::vector<visualization_msgs::Marker> obsVec, int frameid) {
        //calculate the slope
        float m, nume, denom;
        if (p1.point.x != p2.point.x) {
            nume = (p2.point.y - p1.point.y);
            denom = (p2.point.x - p1.point.x);
            m = nume / denom;
        }
        float theta = atan(m);
        if (theta < 0) {
            if (denom < 0) {
                theta = theta + M_PI;
            } else {
                theta = theta + 2 * M_PI;
            }
        } else {
            if ((nume < 0) && (denom < 0)) {
                theta = theta + M_PI;
            }
        }
        float sin_theta = sin(theta);
        float cos_theta = cos(theta);

        //calculate P
        Node p;
        p.point.y = sigma * sin_theta + p1.point.y;
        p.point.x = sigma * cos_theta + p1.point.x;
        p.point.z = 0;
        p.id = frameid;

        // calculate if the point is within an obstacle
        if (!intersectsObs(p1.point, p.point, obsVec) && isWithinWorld(p.point)) {
            std::vector<Node>::iterator it = parentList.begin();
            it = parentList.insert(it, p1);

            p.parentId = p1.id;
            p1.children.push_back(p); //children of init is not in the nodeslist

            nodesList.push_back(p);
            return p;
        }
        return p1;
    }

    bool isWithinWorld(geometry_msgs::Point p) {
        return (p.x > this->x_min && p.x < this->x_max && p.y > this->y_min && p.y < this->y_max);
    }

    bool
    intersectsObs(geometry_msgs::Point p1, geometry_msgs::Point p2, std::vector<visualization_msgs::Marker> obsVec) {

        float x1 = p1.x;
        float y1 = p1.y;
        float x2 = p2.x;
        float y2 = p2.y;

        for (int i = 0; i < obsVec.size(); i++) {
            visualization_msgs::Marker obs = obsVec[i];

            float obs_xl = (obs.pose.position.x - obs.scale.x / 2) - 0.5;
            float obs_xr = (obs.pose.position.x + obs.scale.x / 2) + 0.5;
            float obs_yb = (obs.pose.position.y - obs.scale.y / 2) - 0.5;
            float obs_yt = (obs.pose.position.y + obs.scale.y / 2) + 0.5;

            //check for the bottom intersection
            bool bottom = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xr, obs_yb);
            //left intersect
            bool left = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yb, obs_xl, obs_yt);
            //right intersect
            bool right = lineIntersect(x1, y1, x2, y2, obs_xr, obs_yb, obs_xr, obs_yt);
            //top intersect
            bool top = lineIntersect(x1, y1, x2, y2, obs_xl, obs_yt, obs_xr, obs_yt);

            if (bottom || left || right || top) {
                return true;
            }
        }
        return false;
    }

    bool lineIntersect(float x1, float y1, float x2, float y2, float x3, float y3, float x4, float y4) {

        // calculate the distance to intersection point
        float uA = ((x4 - x3) * (y1 - y3) - (y4 - y3) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));
        float uB = ((x2 - x1) * (y1 - y3) - (y2 - y1) * (x1 - x3)) / ((y4 - y3) * (x2 - x1) - (x4 - x3) * (y2 - y1));

        // if uA and uB are between 0-1, lines are colliding
        if (uA >= 0 && uA <= 1 && uB >= 0 && uB <= 1) {

            float intersectionX = x1 + (uA * (x2 - x1));
            float intersectionY = y1 + (uA * (y2 - y1));

            return true;
        }
        return false;
    }

    std::vector<Node> getNodesList() {
        return this->nodesList;
    }

private:
    Node _init, _goal;
    float sigma;
    int x_max;
    int x_min;
    int y_max;
    int y_min;
    std::vector<Node> nodesList;
    std::vector<Node> parentList;
};
