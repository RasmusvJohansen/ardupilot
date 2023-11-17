/*################################################################################
  ##
  ##   Copyright (C) 2016-2023 Keith O'Hara
  ##
  ##   This file is part of the OptimLib C++ library.
  ##
  ##   Licensed under the Apache License, Version 2.0 (the "License");
  ##   you may not use this file except in compliance with the License.
  ##   You may obtain a copy of the License at
  ##
  ##       http://www.apache.org/licenses/LICENSE-2.0
  ##
  ##   Unless required by applicable law or agreed to in writing, software
  ##   distributed under the License is distributed on an "AS IS" BASIS,
  ##   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  ##   See the License for the specific language governing permissions and
  ##   limitations under the License.
  ##
  ################################################################################*/
#undef _GLIBCXX_USE_C99_STDIO

#ifndef OPTIMLIB_INCLUDES
#define OPTIMLIB_INCLUDES




#include "Code/optim_options.hpp"

namespace optim
{
    // misc/utility files
    #include "Code/optim_misc.hpp"

    // stats/rng files
    #include "Code/optim_stats.hpp"

    // line search
    #include "Code/more_thuente.hpp"

    // unconstrained optimization
    #include "Code/optim_unconstrained.hpp"

    // constrained optimization
    //#include "Code/sumt.hpp"

    // solving systems of nonlinear equations
    #include "Code/optim_zeros.hpp"
}

#endif
