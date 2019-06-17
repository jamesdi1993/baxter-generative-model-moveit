from os import listdir
from os.path import join
import argparse

directory = "./logs/"
metrics_string = {
    "numTotalSamples": "Number of samples collected in total"
}

def stats_from_run(files, metrics):
    results = {}
    for key in metrics:
        results[key] = 0

    for f in files:
        with open(f) as input:
            for line in input:
                text = line.split(": ")
                for metric, value in metrics_string.items():
                    if value == text[0]:  # if the string matches;
                        results[metric] = results.get(metric) + float(text[1])

    # Average
    for key, value in results.items():
        results[key] = value / 10
    return results

if __name__=="__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("-g", "--goal", required=True,
                    help="Goal configs.")
    ap.add_argument("-p", "--planner", required=True,
                    help="Planner to gather statistics for")
    args = vars(ap.parse_args())

    goal = args["goal"]
    planner = args["planner"]

    subdir = ""
    if goal == "fixed":
        subdir = "fixed_goal_10"
    elif goal == "random":
        subdir = "random_goal_10"
    else:
        raise RuntimeError("Unknown argument for goal configurations.")

    if planner == "fmt":
        subdir += "/" + "fmt"
    elif planner == "vaefmt":
        subdir += "/" + "vaefmt"
    else:
        raise RuntimeError("Unknown argument for planner.")

    dir = directory + subdir

    files = [join(dir, f) for f in listdir(dir) if f.endswith(".log")]
    metrics = ["numTotalSamples"]
    stats = stats_from_run(files, metrics)
    print("The result is: %s" % stats)


