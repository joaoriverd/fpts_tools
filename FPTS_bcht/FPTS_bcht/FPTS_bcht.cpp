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
#include <math.h>       // ceil
#include <chrono>
#include <random>
using namespace std;

#define NUMBER_OF_TASKS    5
#define TARGET_TASK_KEY    (NUMBER_OF_TASKS-1)

enum TaskTypeEnum
{
  target_task = 0,
  preemptive_task,
  delaying_task,
  blocking_task,
  inferred_blocking_task,
  non_blocking_task
};

typedef size_t task_key_type;
typedef size_t priority_type;

size_t bcht;

struct task_type
{
  size_t key;
  size_t period;
  size_t computation;
  size_t prio;
  size_t threashold;
  size_t phase;
  size_t wcrt;
  size_t bcrt;
  size_t AJ;
  TaskTypeEnum type;
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

bool is_idle = true;

bool job_compare(const job_type& j1, const job_type& j2) {

  if (j1.parent_task == j2.parent_task)
  {
    return j1.key > j2.key;
  }

  return j1.current_prio < j2.current_prio;

}

bool schedule_next_job(std::vector<job_type>& active_jobs, std::vector<job_type>& preempted_jobs, running_job_type& running_job)
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

void fpts_simulator(std::vector<task_type>& ts, size_t target_task_key)
{
  static std::map<size_t, std::vector<job_type>> released_jobs;
  static std::vector<job_type> active_jobs;
  static std::vector<job_type> preempted_jobs;
  running_job_type running_job;
  size_t simulation_time;

  // We want to simulate during 4 lcm of the periods. Hence, first find the lcm.
  size_t hyper_period = 1;
  for (const task_type& t : ts)
  {
    hyper_period = lcm(hyper_period, t.period);
  }
  simulation_time = 4 * hyper_period + hyper_period / 2;


  // First register all the tasks and plot the release time
  for (task_type& t : ts)
  {
    size_t n = simulation_time / t.period;

    size_t activation_time = t.phase;
    // Iterate over all jobs of task t and plot activation time
    for (size_t k = 0; k < n; k++)
    {
      // register two subjobs for each task instance
      job_type job = { k, t.key, t.prio, t.computation, 0, activation_time };

      released_jobs[activation_time].push_back(job);
      activation_time += t.period;
    }
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
          if (response_time > ts[running_job.parent_task].wcrt)
          {
            ts[running_job.parent_task].wcrt = response_time;
          }

          if (response_time < ts[running_job.parent_task].bcrt)
          {
            ts[running_job.parent_task].bcrt = response_time;
          }
        }

        sched_time -= running_job.remaining_time;
        is_idle = true;

        if (sched_time > 0)
        {
          if (schedule_next_job(active_jobs, preempted_jobs, running_job))
          {
            running_job.restart_time = current_time - sched_time;

            // lift priority to threashold
            running_job.current_prio = ts[running_job.parent_task].threashold;

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
    if (schedule_next_job(active_jobs, preempted_jobs, running_job))
    {
      running_job.restart_time = current_time;

      // lift priority to threashold
      running_job.current_prio = ts[running_job.parent_task].threashold;

      is_idle = false;
    }

    prev_time = current_time;

    // break loop if lower bound found
    if (ts[target_task_key].bcrt == bcht)
    {
      break;
    }
  }

  // Clean all vectors used to prepare for next iteration
  released_jobs.clear();
  active_jobs.clear();
  preempted_jobs.clear();
}

int ceil_div(int n, int d)
{
  int r = n / d;

  if (n%d > 0)
  {
    r++;
  }

  return r;
}

size_t worst_case_hold_time(std::vector<task_type>& ts, size_t t_key)
{
  size_t wcht = ts[t_key].computation;
  size_t wcht_old = 0;
  size_t sum = 0;

  while (wcht != wcht_old)
  {
    wcht_old = wcht;
    sum = 0;
    for (const task_type& t : ts)
    {
      if (t.prio > ts[t_key].threashold)
      {
        sum += ceil_div(wcht, t.period)*t.computation;
      }
    }

    wcht = ts[t_key].computation + sum;
  }

  return wcht;
}

size_t worst_case_response_time(std::vector<task_type>& ts, size_t t_key)
{
  size_t wcht = ts[t_key].computation;
  size_t wcht_old = 0;
  size_t sum = 0;

  while (wcht != wcht_old)
  {
    wcht_old = wcht;
    sum = 0;
    for (const task_type& t : ts)
    {
      if (t.prio > ts[t_key].prio)
      {
        sum += ceil_div(wcht, t.period)*t.computation;
      }
    }

    wcht = ts[t_key].computation + sum;
  }

  return wcht;
}

size_t hold_time_lower_bound(std::vector<task_type>& ts, size_t t_key)
{
  size_t bcht = worst_case_hold_time(ts, t_key);
  size_t bcht_old = std::numeric_limits<size_t>::max();
  size_t sum = 0;
  size_t temp;

  while (bcht != bcht_old)
  {
    bcht_old = bcht;
    sum = 0;
    for (const task_type& t : ts)
    {
      if (t.prio > ts[t_key].threashold)
      {
        temp = ceil_div(bcht, t.period) - 1;
        if (temp > 0)
        {
          sum += temp*t.computation;
        }
      }
    }

    bcht = ts[t_key].computation + sum;
  }

  return bcht;
}

size_t hold_time_upper_bound(std::vector<task_type>& ts, size_t t_key)
{
  size_t bcht = worst_case_response_time(ts, t_key);
  size_t bcht_old = std::numeric_limits<size_t>::max();
  size_t sum = 0;
  int temp;

  while (bcht != bcht_old)
  {
    bcht_old = bcht;
    sum = 0;
    for (const task_type& t : ts)
    {
      if (t.prio > ts[t_key].prio)
      {
        temp = ceil_div(bcht - t.AJ, t.period) - 1;
        if (temp > 0)
        {
          sum += temp*t.computation;
        }
      }
    }

    bcht = ts[t_key].computation + sum;
  }

  return bcht;
}

size_t hold_time_upper_bound_delta(std::vector<task_type>& ts, size_t t_key)
{
  size_t bcht = worst_case_response_time(ts, t_key);
  size_t bcht_old = std::numeric_limits<size_t>::max();
  size_t sum = 0;
  int temp;

  while (bcht != bcht_old)
  {
    bcht_old = bcht;
    sum = 0;
    for (const task_type& t : ts)
    {
      if (t.prio > ts[t_key].threashold)
      {
        temp = ceil_div(bcht, t.period) - 1;
        if (temp > 0)
        {
          sum += temp*t.computation;
        }
      }

      if (ts[t_key].threashold >= t.prio && t.prio > ts[t_key].prio)
      {
        temp = (bcht - t.AJ) / t.period;
        if (temp > 0)
        {
          sum += temp*t.computation;
        }
      }
    }

    bcht = ts[t_key].computation + sum;
  }

  return bcht;
}


void init_taskset(std::vector<task_type>& ts)
{
  task_type t;

  for (size_t i = 0; i < NUMBER_OF_TASKS; i++)
  {
    t.key = i;
    t.prio = NUMBER_OF_TASKS - i;
    t.threashold = t.prio;
    t.AJ = 0;
    ts.push_back(t);
  }

  /*
  ts[0].period = 80;
  ts[0].computation = 20;
  ts[0].type = preemptive_task;
  ts[1].period = 30;
  ts[1].computation = 10;
  ts[1].type = delaying_task;
  ts[2].period = 120;
  ts[2].computation = 50;
  ts[2].threashold = ts[1].prio;
  ts[2].type = target_task;
  */

  /*
  ts[0].period = 80;
  ts[0].computation = 14;
  ts[0].type = preemptive_task;
  ts[1].period = 80;
  ts[1].computation = 6;
  ts[1].type = preemptive_task;
  ts[2].period = 30;
  ts[2].computation = 9;
  ts[2].type = delaying_task;
  ts[3].period = 120;
  ts[3].computation = 54;
  ts[3].threashold = ts[2].prio;
  ts[3].type = target_task;
  */

  ts[0].period = 140;
  ts[0].computation = 20;
  ts[0].type = preemptive_task;
  ts[1].period = 110;
  ts[1].computation = 20;
  ts[1].type = preemptive_task;
  ts[2].period = 60;
  ts[2].computation = 10;
  ts[2].type = delaying_task;
  ts[3].period = 25;
  ts[3].computation = 10;
  ts[3].type = delaying_task;
  ts[4].period = 800;
  ts[4].computation = 80;
  ts[4].threashold = ts[2].prio;
  ts[4].type = target_task;
}

int main(int argc, char** argv) {
  std::vector<task_type> ts;
  size_t ht_lower;
  size_t ht_upper;
  size_t critical_region;
  std::map<size_t, size_t> num_delaying_jobs;
  size_t critical_push_interval;
  size_t job_push_interval;
  size_t bcht_lower_bound;

  // Init taskset
  init_taskset(ts);

  // Calculate hold-time lower bound
  ht_lower = hold_time_lower_bound(ts, TARGET_TASK_KEY);

  // Assign this lowerbound as jitter for delaying tasks, to simulate 
  // non preemption of jobs after start of target task.
  size_t AJ_delaying_tasks = ht_lower; // not sure here if should substract 1
  for (task_type& t : ts)
  {
    if (t.type == delaying_task)
    {
      t.AJ = AJ_delaying_tasks;
    }
  }

  ht_upper = hold_time_upper_bound_delta(ts, TARGET_TASK_KEY);

  // If the hold time upper and lower bounds are not equal, it means that
  // delaying tasks affectthe bcht.
  while (ht_upper > ht_lower)
  {
    // Calculate how many delaying jobs per delaying task affect the lower bound, and
    // derive the delaying task which activation is closer to the start of the critical region
    critical_region = ht_upper - ht_lower;
    critical_push_interval = critical_region;
    for (const task_type& t : ts)
    {
      if (t.type == delaying_task)
      {
        num_delaying_jobs[t.key] = critical_region / t.period;
        job_push_interval = critical_region % t.period;
        if (job_push_interval < critical_push_interval)
        {
          critical_push_interval = job_push_interval;
        }
      }
    }

    // Push all the delaying tasks according to critical_push_interval
    for (task_type& t : ts)
    {
      if (t.type == delaying_task)
      {
        t.AJ = ht_lower + critical_push_interval;
        bcht_lower_bound = t.AJ;
      }
    }

    // Calculate the new upper bound
    ht_upper = hold_time_upper_bound(ts, TARGET_TASK_KEY);
  }

  return 0;
}