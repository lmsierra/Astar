#pragma once

#include <vector>
#include <algorithm>
#include <string>
#include "Vector3D.h"

class PowerUp;
typedef std::vector<PowerUp*> PowerUps;

class MapNode
{
public:
    MapNode(std::string name, const Vector3D& position) : m_name(name), m_position(position) {}
    ~MapNode() {}


    void AddLink(MapNode* node)
    {
        if (!node || node == this)
            return;

        // Avoid adding duplicates
        std::vector<MapNode*>::iterator it = std::find(m_links.begin(), m_links.end(), node);
        if (it != m_links.end())
            return;

        m_links.push_back(node);
    }
    
    void RemoveLink(MapNode *node)
    {
        if (!node)
            return;

        std::vector<MapNode*>::iterator it = std::find(m_links.begin(), m_links.end(), node);
        if (it == m_links.end()) // Check the node was found
            return;

        m_links.erase(it);
    }

    std::string GetName() const
    {
        return m_name;
    }

    const std::vector<MapNode*>& GetLinks() const
    {
        return m_links;
    }

    const Vector3D& GetPosition() const
    {
        return m_position;
    }

protected:
    Vector3D              m_position;
    std::string           m_name;
    std::vector<MapNode*> m_links;
};
