/* 
 * Copyright (c) 2018-2019, CNRS-UM LIRMM
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <capture_walking/HorizontalMPCProblem.h>

namespace capture_walking
{
  using namespace HorizontalMPC;

  void HorizontalMPCProblem::writePython(const std::string & suffix)
  {
    auto t = std::time(nullptr);
    auto tm = std::localtime(&t);
    std::stringstream fileName;
    fileName << "/tmp/lcp_plot"
      << "_" << 10000 + (++nbWritePythonCalls_) << "_"
      << "ss" << nbInitSupportSteps_ << "-"
      << "ds" << nbDoubleSupportSteps_ << "-"
      << "ts" << nbTargetSupportSteps_ << "-"
      << "nds" << nbNextDoubleSupportSteps_
      << ((suffix.length() > 0) ? "_" : "") << suffix
      << ".py";

    pyScript_.open(fileName.str());
    pyScript_.precision(20);
    pyScript_ << "#!/usr/bin/env python" << std::endl << std::endl
      << "import IPython" << std::endl << std::endl
      << "from pylab import *" << std::endl << std::endl
      << "n = " << NB_STEPS + 1 << std::endl
      << "nb_init_support_steps = " << nbInitSupportSteps_ << std::endl
      << "nb_double_support_steps = " << nbDoubleSupportSteps_ << std::endl
      << "nb_target_support_steps = " << nbTargetSupportSteps_ << std::endl
      << "nb_next_double_support_steps = " << nbNextDoubleSupportSteps_ << std::endl
      << "t = [i * " << SAMPLING_PERIOD << " for i in xrange(" << NB_STEPS + 1 << ")]" << std::endl;
    writePythonContact(initContact_, "init_contact");
    writePythonContact(targetContact_, "target_contact");
    writePythonSolution();

    Eigen::Vector2d dcmRef = targetContact_.p().head<2>();
    pyScript_ << "zeta = " << zeta_ << std::endl
      << "dcm_ref = [" << dcmRef(0) << ", " << dcmRef(1) << "]"
      << std::endl;

    pyScript_ << R"(
assert len(com_x) == n
assert len(u_x) == n - 1

omegaInv = sqrt(zeta)
dcm_x = [com_x[i] + omegaInv * comd_x[i] for i in xrange(n)]
dcm_y = [com_y[i] + omegaInv * comd_y[i] for i in xrange(n)]
zmp_x = [com_x[i] - zeta * comdd_x[i] for i in xrange(n)]
zmp_y = [com_y[i] - zeta * comdd_y[i] for i in xrange(n)]

com = [array([com_x[i], com_y[i]]) for i in xrange(n)]
comd = [array([comd_x[i], comd_y[i]]) for i in xrange(n)]
comdd = [array([comdd_x[i], comdd_y[i]]) for i in xrange(n)]
comddd = [array([u_x[i], u_y[i]]) for i in xrange(n - 1)]
dcm = [array([dcm_x[i], dcm_y[i]]) for i in xrange(n)]
zmp = [array([zmp_x[i], zmp_y[i]]) for i in xrange(n)]
zmp_ref = [array([zmp_ref_x[i], zmp_ref_y[i]]) for i in xrange(n)]

zmp_error = [zmp[i] - zmp_ref[i] for i in xrange(n)]

def plot_contact(contact):
    plot([x for (x, y, z) in contact], [y for (x, y, z) in contact], 'k--')

def plot_x_y(fig_id):
    figure(fig_id)
    plot(com_x, com_y, 'b-')
    plot(dcm_x, dcm_y, 'g-')
    plot(zmp_x, zmp_y, 'r--')
    plot(zmp_ref_x, zmp_ref_y, 'k:')
    legend(('CoM', 'DCM', 'ZMP', 'ZMP_ref'))
    xlim(-0.1, 0.4)
    ylim(-0.2, 0.3)
    plot_contact(init_contact)
    plot_contact(target_contact)
    plot([com_x[0]], [com_y[0]], 'bo', markersize=15)
    plot([dcm_ref[0]], [dcm_ref[1]], 'g*', markersize=20)
    plot([dcm_x[0]], [dcm_y[0]], 'go', markersize=15)
    plot([zmp_ref_x[0]], [zmp_ref_y[0]], 'ko', markersize=15)
    plot([zmp_x[0]], [zmp_y[0]], 'ro', markersize=15)
    plot(com_x, com_y, 'bo')
    plot(dcm_x, dcm_y, 'go')
    plot(zmp_ref_x, zmp_ref_y, 'ko')
    plot(zmp_x, zmp_y, 'ro')

def plot_t_xy(fig_id):
    figure(fig_id)
    subplot(311)
    plot(t, com_x, 'b-')
    plot(t, dcm_x, 'g-')
    plot(t, zmp_x, 'r-')
    plot(t, zmp_ref_x, 'r--')
    legend(('COM_x', 'DCM_x', 'ZMP_x', 'ZMP_ref_x'))
    plot(t, x_min, 'k:', drawstyle='steps-post')
    plot(t, x_max, 'k:', drawstyle='steps-post')
    subplot(312)
    plot(t, com_y, 'b-')
    plot(t, dcm_y, 'g-')
    plot(t, zmp_y, 'r-')
    plot(t, zmp_ref_y, 'r--')
    legend(('COM_y', 'DCM_y', 'ZMP_y', 'ZMP_ref_y'))
    plot(t, y_min, 'k:', drawstyle='steps-post')
    plot(t, y_max, 'k:', drawstyle='steps-post')
    subplot(313)
    plot(t, vel_ref_x, 'r--')
    plot(t, vel_ref_y, 'g--')
    legend(('refVel_x', 'refVel_y'))

if __name__ == "__main__":
    ion()
    # close('all')
    clf()
    plot_t_xy(1)
    # plot_x_y(2)
    if IPython.get_ipython() is None:
        IPython.embed()

# File written: )";

    pyScript_ //<< (1900 + tm->tm_year) << "/"
      << (1 + tm->tm_mon) << "/" << tm->tm_mday
      << " " << tm->tm_hour << ":" << tm->tm_min << ":" << tm->tm_sec;
    pyScript_.close();
  }

  void HorizontalMPCProblem::writePythonContact(const Contact & contact, const std::string & label)
  {
    pyScript_ << label << " = ["
      << "(" << contact.vertex0()(0) << ", " << contact.vertex0()(1) << ", " << contact.vertex0()(2) << "), "
      << "(" << contact.vertex1()(0) << ", " << contact.vertex1()(1) << ", " << contact.vertex1()(2) << "), "
      << "(" << contact.vertex2()(0) << ", " << contact.vertex2()(1) << ", " << contact.vertex2()(2) << "), "
      << "(" << contact.vertex3()(0) << ", " << contact.vertex3()(1) << ", " << contact.vertex3()(2) << "),"
      << "(" << contact.vertex0()(0) << ", " << contact.vertex0()(1) << ", " << contact.vertex0()(2) << ")]"
      << std::endl;
  }

  void HorizontalMPCProblem::writePythonSolution()
  {
    const Eigen::VectorXd & stateTraj = solution_.stateTraj();
    const Eigen::VectorXd & jerkTraj = solution_.jerkTraj();
    writePythonSerializedVector(jerkTraj, "u_x", 0, NB_STEPS);
    writePythonSerializedVector(jerkTraj, "u_y", 1, NB_STEPS);
    writePythonSerializedVector(stateTraj, "com_x", 0, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "com_y", 1, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comd_x", 2, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comd_y", 3, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comdd_x", 4, NB_STEPS + 1);
    writePythonSerializedVector(stateTraj, "comdd_y", 5, NB_STEPS + 1);
    writePythonSerializedVector(zmpRef_, "zmp_ref_x", 0, NB_STEPS + 1);
    writePythonSerializedVector(zmpRef_, "zmp_ref_y", 1, NB_STEPS + 1);
    writePythonSerializedVector(velRef_, "vel_ref_x", 0, NB_STEPS + 1);
    writePythonSerializedVector(velRef_, "vel_ref_y", 1, NB_STEPS + 1);
  }

  void HorizontalMPCProblem::writePythonSerializedVector(const Eigen::VectorXd & vec, const std::string & label, unsigned index, unsigned nbChunks)
  {
    long chunkSize = vec.size() / nbChunks;
    pyScript_ << label << " = [";
    for (unsigned i = 0; i < nbChunks; i++)
    {
      pyScript_ << vec(chunkSize * i + index);
      if (i < nbChunks - 1)
      {
        pyScript_ << ", ";
      }
    }
    pyScript_ << "]" << std::endl;
  }
}
