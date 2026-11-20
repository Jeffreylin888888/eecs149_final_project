# Third-party notices and provenance

This product includes software developed by Cyber-Physical & Systems Lab at UCLA.

The planner's mathematical method follows Shoukry et al., *Linear Temporal Logic Motion Planning for Teams of Underactuated Robots Using Satisfiability Modulo Convex Programming*, IEEE CDC 2017, DOI: https://doi.org/10.1109/CDC.2017.8263808. The legacy-workspace generator adapts the grid geometry from `examples/LTL Motion Planning/ScalabilityLTLMotionPlanning.py` in the authors' SatEX archive, downloaded from https://yshoukry.bitbucket.io/SatEX/SatEX.zip. Its complete original source notice is preserved below. Obsolete solver binaries from the original archive are not distributed in this project.

```text
#  * --------------------------------------------------------------------------
#  * File: ScalabilityLTLMotionPlanning.py
#  * ---------------------------------------------------------------------------
#  * Copyright (c) 2016 The Regents of the University of California.
#  * All rights reserved.
#  *
#  * Redistribution and use in source and binary forms, with or without
#  * modification, are permitted provided that the following conditions
#  * are met:
#  * 1. Redistributions of source code must retain the above copyright
#  *    notice, this list of conditions and the following disclaimer.
#  * 2. Redistributions in binary form must reproduce the above
#  *    copyright notice, this list of conditions and the following
#  *    disclaimer in the documentation and/or other materials provided
#  *    with the distribution.
#  * 3. All advertising materials mentioning features or use of this
#  *    software must display the following acknowledgement:
#  *       This product includes software developed by Cyber-Physical &
#  *       Systems Lab at UCLA.
#  * 4. Neither the name of the University nor that of the Laboratory
#  *    may be used to endorse or promote products derived from this
#  *    software without specific prior written permission.
#  *
#  * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS''
#  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
#  * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
#  * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE REGENTS
#  * OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#  * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
#  * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
#  * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
#  * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
#  * SUCH DAMAGE.
#  *
#  * Developed by: Yasser Shoukry
#  */
```

Runtime dependencies are installed separately under their respective upstream licenses: NumPy, SciPy (including its HiGHS backend), Z3, and Matplotlib. This notice does not replace their license files. The new implementation does not claim endorsement by the authors, institutions, or upstream solver maintainers.
