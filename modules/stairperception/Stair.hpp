/*
 * Stair.hpp
 *
 *  Created on: 2017年10月6日
 *      Author: zxm
 */

#ifndef STAIR_HPP_
#define STAIR_HPP_

#include "common.h"

// declare
class Step;
class ConcaveLine;

typedef enum
{
    concaveline_point,
    step_point,
    node_point
} PNextType;

class Node {
public:
    PNextType pnext_type;

    Node* pnext;
};

class Line
{
public:
    Line():h(0),d(0){};

    pcl::ModelCoefficients coeff;
    float h,d;
};

class Step:Node
{
public:
    Step():height(0),depth(0),count(0){}

    Plane *plane_v,*plane_h;
    Line line;
    int count;
    double height,depth;
};

class ConcaveLine:Node
{
public:
    Plane *plane_v,*plane_h;
    Line line;
};

class Stair
{
public:
    Stair():step_count(0)
    {
        phead = new Node;
        phead->pnext = NULL;

        pend = new Node;
        pend->pnext = NULL;

        point_current = phead;
    }

    ~Stair()
    {
//         Node* pnext;
//         while(point_current->pnext != NULL)
//         {
//             pnext = point_current->pnext;
//             delete point_current;
//             point_current = pnext;
//         }
//         delete pend;
    }

    void pushBack(Node *pnode, PNextType point_type)
    {
        point_current->pnext_type = point_type;
        point_current->pnext = pnode;
        point_current = pnode;
    }

    void pushBackEnd()
    {
        point_current->pnext_type = node_point;
        point_current->pnext = pend;
        point_current = pend;
    }

    Node* getHead()
    {
        return phead;
    }

    Node* getCurPoint()
    {
        return point_current;
    }

private:
    Node *phead,*pend;
    Node *point_current;
    int step_count;
};



#endif /* STAIR_HPP_ */
