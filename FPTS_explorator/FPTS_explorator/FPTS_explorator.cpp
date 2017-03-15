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

// Definitions for bounds of periods and utilization.
#define PERIOD_GRAIN 5
#define UTIL_GRAIN 1000
#define MAX_UTIL 990
#define LOWER_BOUND_PERIOD 10
#define UPPER_BOUND_PERIOD 80
#define LOWER_BOUND_UTILIZATION 100
#define UPPER_BOUND_UTILIZATION 400

// Charachteristics of task-set to analyse
#define NUM_PREEMPTIVE_TASKS 1
#define NUM_DELAYING_TASKS   0
#define NUM_BLOCKING_TASKS   1
#define NUM_INFER_BLOCKING_TASKS   0   // The number of blocking tasks shoud be non-zero for this field to become non-zero
#define NUM_NON_BLOCKING_TASKS     0    
#define NUMBER_OF_TASKS    (NUM_PREEMPTIVE_TASKS+NUM_DELAYING_TASKS+NUM_BLOCKING_TASKS+\
                           NUM_INFER_BLOCKING_TASKS+NUM_NON_BLOCKING_TASKS+1) 
#define TARGET_TASK_ID     (NUM_PREEMPTIVE_TASKS + NUM_DELAYING_TASKS)
#define TRH_TARGET_TASK    (NUMBER_OF_TASKS - NUM_PREEMPTIVE_TASKS)

// Decide which mode to use to explore non-trivial example. 
// Options. BCHT: Uses the bcht of the task set and compares it with the job experiencing hypothetical optimal instant 
#define BCHT 0
#define IGNORING_INFERRED_BLOCKING 1
#define IGNORING_LOWER_PRIO_TASKS 2
#define IGNORING_DELAYING_TASKS 3
#define EXPLORATION_MODE IGNORING_DELAYING_TASKS

// Change phase of target task (only) in the simulation to search shortest response time (it may slow the exploration)
#define CHANGE_PHASE_TARGET_TASK true

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

size_t ceil_div(size_t n, size_t d)
{
  size_t r = n / d;

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

size_t best_case_hold_time(std::vector<task_type>& ts, size_t t_key)
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

void print_taskset(std::vector<task_type>& ts)
{
  size_t lcm_periods = 1;
  for (const task_type& t : ts)
  {
    lcm_periods = lcm(lcm_periods, t.period);
  }

  std::cout << "Non-trivial taskset found for the last task. lcm = " << lcm_periods << "\n";

  // Print charachteristics of task set
  std::cout << "Task | Period | Exec. | Prio | Thrs | Phase | BCRT | WCRT\n";
  std::cout << "------------------------------------------------\n";
  for (const task_type& t : ts)
  {
    std::cout << " " << t.key << "   |   " << t.period << "   |  " << t.computation << "   |  " << t.prio << "   |  ";
    std::cout << t.threashold << "   |  " << t.phase << "   |  " << t.bcrt << "   |  " << t.wcrt << "\n";
  }
  std::cout << "------------------------------------------------\n\n";
}

size_t get_random_number(size_t lower_bound, size_t upper_bound)
{
  static unsigned seed1 = std::chrono::system_clock::now().time_since_epoch().count();
  static std::default_random_engine e(seed1);
  std::uniform_int_distribution <size_t> d(lower_bound, upper_bound);
  return d(e);
}

void generate_random_taskset(std::vector<task_type>& ts)
{
  task_type t1 = { 0,0,0,0,0,0,0,std::numeric_limits<size_t>::max() };
  double upper_bound_util;
  double upper_bound_period;
  double cummulative_utilization;
  double lower_bound_period;
  bool valid_task_set = false;
  size_t key;

  // Initialise keys, and priorities of all tasks
  for (key = 0; key < NUMBER_OF_TASKS; key++)
  {
    t1.key = key;
    t1.prio = NUMBER_OF_TASKS - key;
    ts.push_back(t1);
  }

  // Initialise all preemptive tasks
  key = 0;
  for (size_t i = 0; i < NUM_PREEMPTIVE_TASKS; i++)
  {
    ts[key].threashold = ts[key].prio;
    ts[key].type = preemptive_task;
    key++;
  }

  // Initialise all delaying tasks
  for (size_t i = 0; i < NUM_DELAYING_TASKS; i++)
  {
    ts[key].threashold = ts[key].prio;
    ts[key].type = delaying_task;
    key++;
  }

  // Initialise target task
  ts[key].threashold = TRH_TARGET_TASK;
  ts[key].type = target_task;
  key++;

  // Initialise all blocking tasks
  for (size_t i = 0; i < NUM_BLOCKING_TASKS; i++)
  {
    ts[key].threashold = ts[TARGET_TASK_ID].prio;
    ts[key].type = blocking_task;
    key++;
  }

  // Initialise all inferred blocking tasks
  for (size_t i = 0; i < NUM_INFER_BLOCKING_TASKS; i++)
  {
    ts[key].threashold = ts[TARGET_TASK_ID].prio - 1; // this tasks will block the blocking tasks of the target task
    ts[key].type = inferred_blocking_task;
    key++;
  }

  // Initialise all non-blocking tasks
  for (size_t i = 0; i < NUM_NON_BLOCKING_TASKS; i++)
  {
    ts[key].threashold = ts[key].prio;
    ts[key].type = non_blocking_task;
    key++;
  }

  // Initialise randomly period and execution time for all the tasks
  while (!valid_task_set)
  {
    cummulative_utilization = 0;
    for (task_type& t : ts)
    {
      t.period = get_random_number(LOWER_BOUND_PERIOD, UPPER_BOUND_PERIOD);
      t.period -= t.period % PERIOD_GRAIN;
      if (t.key != ts.back().key)
      {
        upper_bound_util = min((double)UPPER_BOUND_UTILIZATION, MAX_UTIL - cummulative_utilization);
        upper_bound_period = t.period*upper_bound_util / UTIL_GRAIN;
        lower_bound_period = t.period*LOWER_BOUND_UTILIZATION / UTIL_GRAIN;
        if (upper_bound_period > lower_bound_period)  // todo: this is a workaround for the bug. A proper fix should be implemented. 
        {
          t.computation = get_random_number(lower_bound_period, upper_bound_period);
        }
        else
        {
          // break the loop and re-make the task-set
          break;
        }
      }
      else
      {
        // If it is the last task, assign as much as utilization possible
        t.computation = t.period * (MAX_UTIL - cummulative_utilization) / UTIL_GRAIN;
      }

      cummulative_utilization += t.computation * UTIL_GRAIN / t.period;
    }

    if (ts.back().computation > 0)
    {
      valid_task_set = true;
    }

  }
}

void fpts_simulator_changing_phases(std::vector<task_type>& ts)
{
  size_t bcrt_temp = std::numeric_limits<size_t>::max();
  size_t phase_temp_target = 0;
  size_t phase_temp_preempt = 0;
  size_t target_key;

  // Search target task
  for (task_type t : ts)
  {
    if (t.type == target_task)
    {
      target_key = t.key;
    }
  }

  size_t& target_phase = ts[target_key].phase;
  size_t& preemptive_phase = ts[0].phase; // this assumes that it is at least one preemptive task

  for (target_phase = 0; target_phase < ts[target_key].period; target_phase++)
  {
    for (preemptive_phase = 0; preemptive_phase < ts[0].period; preemptive_phase++)
    {
      // Run simulator
      fpts_simulator(ts, target_key);

      // Keep track of the phase where the shortest response time was found
      if (ts[target_key].bcrt < bcrt_temp)
      {
        bcrt_temp = ts[target_key].bcrt;
        phase_temp_target = target_phase;
        phase_temp_preempt = preemptive_phase;
      }

      // Break loop if lower bound found
      if (ts[target_key].bcrt == bcht)
      {
        break;
      }
    }

    // Break loop if lower bound found
    if (ts[target_key].bcrt == bcht)
    {
      break;
    }
  }
  target_phase = phase_temp_target;
  preemptive_phase = phase_temp_preempt;
}

int main(int argc, char** argv) {
  std::vector<task_type> task_set;
  std::vector<task_type> task_set2;
  size_t target_key2;
  //size_t bcht;
  bool non_trivial_taskset = false;

  // Find a non trivial example where the bcrt of the last task is not equal to its bcht
  while (true)
  {
    generate_random_taskset(task_set);

    // Calculate the trivial best case hold time of the target task
    bcht = best_case_hold_time(task_set, TARGET_TASK_ID);

    // Force phases to hypothetical optimal instant of task to analyse
    for (task_type& t : task_set)
    {
      // Phases for preemptive tasks t_h
      if (t.prio > task_set[TARGET_TASK_ID].threashold)
      {
        t.phase = (bcht % t.period);
      }
      else if (t.prio > task_set[TARGET_TASK_ID].prio && task_set[TARGET_TASK_ID].threashold >= t.prio)
      {
        // Phases for delaying tasks t_d
        t.phase = 1;
      }
      else if (task_set[TARGET_TASK_ID].prio >= t.prio && t.threashold >= task_set[TARGET_TASK_ID].prio)
      {
        // Phases for blocking tasks t_b
        t.phase = 0;
      }
      else
      {
        t.phase = 0;
      }
    }

#if CHANGE_PHASE_TARGET_TASK == false

    // Run simulator
    fpts_simulator(task_set);
#else
    fpts_simulator_changing_phases(task_set);
#endif

#if EXPLORATION_MODE == BCHT
    non_trivial_taskset = task_set[TARGET_TASK_ID].bcrt != bcht;
#else
    // Add all tasks exept inferred blocking tasks to task_set2
    size_t key = 0;
    for (task_type t : task_set)
    {
      if (t.type != delaying_task)
      {
        t.key = key;
        key++;
        task_set2.push_back(t);
      }
      if (t.type == target_task)
      {
        target_key2 = t.key;
      }
    }

    // Run simulator for this new task_set
    fpts_simulator_changing_phases(task_set2);

    non_trivial_taskset = task_set[TARGET_TASK_ID].bcrt != task_set2[target_key2].bcrt;
#endif

    if (non_trivial_taskset)
    {
      // Non trival task set found. Print such a taskset.
      print_taskset(task_set);

      //reset flag
      non_trivial_taskset = false;
    }

    // Clear task set to prepare for next iteration
    task_set.clear();
    task_set2.clear();
  }

  return 0;
}