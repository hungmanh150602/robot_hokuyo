#pragma once

#include "leg_detector/types.hpp"
#include "person.hpp"

std::vector<Person>
detectPersons(
    std::vector<Cluster>& clusters);