// Control_tasks_tool.cpp 
//
#include "stdafx.h"
#include <limits>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <algorithm>    // std::sort
using namespace std;

#define SIM_STEP 1

typedef size_t task_key_type;
typedef size_t priority_type;

struct task_type
{
  size_t key;
  size_t period;
  size_t computation;
  size_t deadline;
  size_t prio;
  size_t threashold;
  size_t phase;
  size_t wcrt;
  size_t bcrt;
};

struct job_type
{
  size_t key;
  task_key_type parent_task;
  size_t current_prio;
  size_t remaining_time;
  size_t restart_time;
  size_t activation_time;
};

typedef job_type running_job_type;

std::map<size_t, std::vector<job_type>> released_jobs;
std::vector<job_type> active_jobs;
std::vector<job_type> preempted_jobs;
running_job_type running_job;
size_t simulation_time;
std::vector<task_type> task_set;
bool is_idle = true;

bool job_compare(const job_type& j1, const job_type& j2) {

  if (j1.parent_task == j2.parent_task)
  {
    return j1.key > j2.key;
  }

  return j1.current_prio < j2.current_prio;

}

void parse_file(std::ifstream& myfile)
{
  if (myfile.is_open())
  {
    string line;

    // Ignore the next line.
    myfile.ignore(256, '#');

    while (false == myfile.eof())
    {
      task_type task;
      myfile >> skipws >> task.period;
      myfile >> skipws >> task.computation;
      myfile >> skipws >> task.deadline;
      myfile >> skipws >> task.prio;
      myfile >> skipws >> task.threashold;
      //myfile >> skipws >> task.phase;

      task_set.push_back(task);
    }

    myfile.close();
  }
  else
  {
    std::cout << "Unable to open file\n";
  }

}

bool schedule_next_job()
{
  std::vector<job_type>* vector_ptr = NULL;
  std::sort(active_jobs.begin(), active_jobs.end(), job_compare);
  std::sort(preempted_jobs.begin(), preempted_jobs.end(), job_compare);

  job_type job_temp = { 0,0,0,0,0,0 };

  if (active_jobs.size() > 0)
  {
    job_temp = active_jobs.back();
    vector_ptr = &active_jobs;
  }

  if (preempted_jobs.size() > 0)
  {
    if (preempted_jobs.back().current_prio >= job_temp.current_prio)
    {
      job_temp = preempted_jobs.back();
      vector_ptr = &preempted_jobs;
    }
  }

  if (NULL == vector_ptr)
  {
    return false;
  }

  if (is_idle == true)
  {
    vector_ptr->pop_back();
    running_job = job_temp;

    return true;
  }

  if (job_temp.current_prio > running_job.current_prio)
  {
    // We have to preempt the running task
    vector_ptr->pop_back();
    preempted_jobs.push_back(running_job);
    running_job = job_temp;

    return true;
  }

  return false;
}

size_t lcm(size_t a, size_t b)
{
  size_t m, n;

  m = a;
  n = b;

  while (m != n)
  {
    if (m < n)
    {
      m = m + a;
    }
    else
    {
      n = n + b;
    }
  }

  return m;
}

int main(int argc, char** argv) {
  //std::ofstream grasp_file;

  // Check arguments
  if (argc < 2)
  {
    std::cout << "Error: You should specify an input task set.\n";
    return 0;
  }

  std::ifstream myfile(argv[1]);

  // Parse input file
  parse_file(myfile);

  // Check that taskset is not empty
  if (task_set.size() == 0)
  {
    cout << "No task set read.\n";
    return 0;
  }

  // Check utilization of the task set
  double utilization = 0;
  for (const task_type& t : task_set)
  {
    utilization += ((t.computation) * 100) / t.period;
  }

  if (utilization > 100)
  {
    cout << "Utilization is over 1.\n";
    //return 0;
  }

  //grasp_file.open(argv[2]);

  // We want to simulate during 4 lcm of the periods. Hence, first find the lcm.
  size_t hyper_period = 1;
  for (const task_type& t : task_set)
  {
    hyper_period = lcm(hyper_period, t.period);
  }
  simulation_time = 4*hyper_period + hyper_period/2;

  // Initialise all the tasks
  for (task_type& t : task_set)
  {
    t.phase = 0;
    t.wcrt = 0;
    t.bcrt = std::numeric_limits<size_t>::max();
  }

  // Iterate until the last task changes its phase.
  while (task_set.back().phase == 0)
  {
    // First register all the tasks and plot the release time
    size_t task_key = 0;
    for (task_type& t : task_set)
    {
      t.key = task_key;
      //grasp_file << "newTask task" << t.key << " -priority " << t.key << " -name \"Task " << t.key+1 << "\"\n";

      size_t n = simulation_time / t.period;

      size_t activation_time = t.phase;
      // Iterate over all jobs of task t and plot activation time
      for (size_t k = 0; k < n; k++)
      {
        //grasp_file << "plot " << activation_time << " jobArrived job" << t.key << "." << k << " task" << t.key << "\n";

        // register two subjobs for each task instance
        job_type job = { k, t.key, t.prio, t.computation, 0, activation_time };

        released_jobs[activation_time].push_back(job);
        activation_time += t.period;
      }

      task_key++;
    }

    // Schedule all the jobs
    running_job = { 0,0,0,0,0,0 };
    size_t current_time = 0;
    size_t prev_time = 0;
    size_t sched_time = 0;
    for (std::pair<size_t, std::vector<job_type>> new_jobs : released_jobs)
    {
      current_time = new_jobs.first;
      sched_time = current_time - prev_time;

      // First complete the schedule in the time period [prev_time, current_time)
      while (sched_time > 0)
      {
        if (running_job.remaining_time <= sched_time && false == is_idle)
        {
          // The running job ends, schedule next job.
          size_t finalization_time = running_job.restart_time + running_job.remaining_time;
          size_t response_time = finalization_time - running_job.activation_time;

          if (current_time > 2 * hyper_period && current_time <= 4 * hyper_period)
          {
            if (response_time > task_set[running_job.parent_task].wcrt)
            {
              task_set[running_job.parent_task].wcrt = response_time;
            }

            if (response_time < task_set[running_job.parent_task].bcrt)
            {
              task_set[running_job.parent_task].bcrt = response_time;
            }
          }

          //grasp_file << "plot " << finalization_time << " message \"Response time job" << running_job.parent_task << "." << running_job.key;
          //grasp_file << ": " << response_time << "\" -color blue -shape square\n";
          //grasp_file << "plot " << finalization_time << " jobCompleted job" << running_job.parent_task << "." << running_job.key;
          sched_time -= running_job.remaining_time;
          is_idle = true;

          if (sched_time > 0)
          {
            if (schedule_next_job())
            {
              running_job.restart_time = current_time - sched_time;
              //grasp_file << " -target job" << running_job.parent_task << "." << running_job.key << "\n";
              //grasp_file << "plot " << running_job.restart_time << " jobResumed job" << running_job.parent_task << "." << running_job.key << "\n";

              // lift priority to threashold
              running_job.current_prio = task_set[running_job.parent_task].threashold;

              is_idle = false;
            }
          }

          //grasp_file << "\n";
        }
        else
        {
          // The running job has not ended yet.
          running_job.remaining_time -= sched_time;
          running_job.restart_time += sched_time;
          sched_time = 0;
        }
      }

      // Add the released jobs to active jobs and order by priorities
      active_jobs.insert(active_jobs.end(), new_jobs.second.begin(), new_jobs.second.end());

      // Decide which job to schedule between the running job, the active jobs and the preemented jobs.
      bool was_idle = is_idle;
      running_job_type prev_job = running_job;
      if (schedule_next_job())
      {
        running_job.restart_time = current_time;
        if (false == was_idle)
        {
          //grasp_file << "plot " << current_time << " jobPreempted job" << prev_job.parent_task << "." << prev_job.key;
          //grasp_file << " -target job" << running_job.parent_task << "." << running_job.key << "\n";
        }
        //grasp_file << "plot " << current_time << " jobResumed job" << running_job.parent_task << "." << running_job.key << "\n";

        // lift priority to threashold
        running_job.current_prio = task_set[running_job.parent_task].threashold;

        is_idle = false;
      }

      prev_time = current_time;
    }

    // Clean all vectors used to prepare for next iteration
    released_jobs.clear();
    active_jobs.clear();
    preempted_jobs.clear();

    // Modify the phase for next iteration
    size_t carry = SIM_STEP;
    for (task_type& t : task_set)
    {
      t.phase += carry;
      carry = (t.phase / t.period)*SIM_STEP;
      t.phase = t.phase%t.period;
    }
  }
  // Print BCRT and WCRT found in the simulation.
  std::cout << "Task | BCRT | WCRT\n";
  std::cout << "------------------\n";
  for (const task_type& t : task_set)
  {
    std::cout << " " << t.key << "   |  " << t.bcrt << "   |  " << t.wcrt << "\n";
  } 

  //grasp_file.close();
  return 0;
}