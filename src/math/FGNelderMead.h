/*
 * FGNelderMead.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * FGNelderMead.h is free software: you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as published by the
 * Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * FGNelderMead.h is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef JSBSim_FGNelderMead_H
#define JSBSim_FGNelderMead_H

#include <vector>
#include <limits>
#include <cstddef>
#include <random>
#include "JSBSim_API.h"

namespace JSBSim
{

class JSBSIM_API FGNelderMead
{
public:
    class Function
    {
    public:
        virtual double eval(const std::vector<double> & v)  = 0;
        virtual ~Function() = default;
    };
    class Callback
    {
    public:
        virtual void eval(const std::vector<double> & v)  = 0;
        virtual ~Callback() = default;
    };

    FGNelderMead(Function * f, const std::vector<double> & initialGuess,
                 const std::vector<double> & lowerBound,
                 const std::vector<double> & upperBound,
                 const std::vector<double> & initialStepSize, int iterMax=2000,
                 double rtol=std::numeric_limits<float>::epsilon(),
                 double abstol=std::numeric_limits<float>::epsilon(),
                 double speed = 2.0,
                 double randomization=0.1,
                 bool showConvergeStatus=true,bool showSimplex=false,
                 bool pause=false,
                 Callback * callback=nullptr);
    std::vector<double> getSolution();

    void update();
    int status();

private:
    // attributes
    Function * m_f = nullptr;
    Callback * m_callback = nullptr;
    double m_randomization = 0.0;
    const std::vector<double> & m_lowerBound;
    const std::vector<double> & m_upperBound;
    size_t m_nDim = 0, m_nVert = 0;
    int m_iMax = 0, m_iNextMax = 0, m_iMin = 0;
    std::vector< std::vector<double> > m_simplex;
    std::vector<double> m_cost;
    std::vector<double> m_elemSum;
    int m_status = 0;
    const std::vector<double> & initialGuess;
    const std::vector<double> & initialStepSize;
    int iterMax = 0, iter = 0;
    double rtol = 0.0, abstol = 0.0, speed = 0.0;
    bool showConvergeStatus = false, showSimplex = false, pause = false;
    double rtolI = 0.0, minCostPrevResize = 0.0, minCost = 0.0, minCostPrev = 0.0,
           maxCost = 0.0, nextMaxCost = 0.0;
    mutable std::mt19937 m_rng{std::random_device{}()};
    mutable std::uniform_real_distribution<double> m_dist{-1.0, 1.0};

    // methods
    double getRandomFactor();
    double tryStretch(double factor);
    void contract();
    void constructSimplex(const std::vector<double> & guess, const std::vector<double> & stepSize);
    void boundVertex(std::vector<double> & vertex,
                     const std::vector<double> & upperBound,
                     const std::vector<double> & lowerBound);
    double eval(const std::vector<double> & vertex, bool check = false);
};

} // JSBSim

#endif

// vim:ts=4:sw=4
