#pragma once

#include "person_detector/types.hpp"

std::vector<Person> detectPersons(std::vector<Cluster>& clusters);