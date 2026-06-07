from pymoo.util.display.column import Column
from pymoo.util.display.output import Output
from PathSolution import *
from PathOptimizationModel import *
from PathProblem import *
from pymoo.util.display.single import MinimumConstraintViolation, AverageConstraintViolation
from pymoo.util.display.multi import NumberOfNondominatedSolutions
from pymoo.util.display.output import Output, pareto_front_if_possible
from pymoo.termination.ftol import MultiObjectiveSpaceTermination
from pymoo.indicators.gd import GD
from pymoo.indicators.hv import Hypervolume
from pymoo.indicators.igd import IGD
from Connectivity import *

class PathOutput(Output):

    def __init__(self, problem:PathProblem):
        super().__init__()

        self.problem = problem

        objs = problem.model["F"]

        self.best_dist = None
        self.best_time = None
        self.time_at_best_conn = None
        self.conn_at_best_time = None
        self.best_subtour = None
        self.best_conn = None
        self.best_mean_disconn = None
        self.best_max_disconn = None
        self.best_disconn = None
        self.max_mean_tbv_info = None
        self.best_sv = None

        width = 13

        for obj in objs:

            if "Total Distance" in obj:
                # print("Distance Output In !")
                self.best_dist = Column("best_dist", width=width)
                # self.min_dist = Column("min_dist", width=13)
                # self.max_dist = Column("max_dist", width=13)
                # self.mean_dist = Column("mean_dist", width=13)
                # self.min_dist = Column("min_dist", width=len("min_dist"))
                # self.max_dist = Column("max_dist", width=len("max_dist"))
                # self.mean_dist = Column("mean_dist", width=len("mean_dist"))
                self.columns += [self.best_dist]

            if "Mission Time" in obj:
                self.best_time = Column("best_time", width=width)
                self.columns += [self.best_time]

            if "Percentage Connectivity" in obj:
                self.best_conn = Column("best_conn", width=width)
                self.columns += [self.best_conn]

            if "Mission Time" in obj and "Percentage Connectivity" in obj:
                self.time_at_best_conn = Column("time_at_best_conn", width=21)
                self.conn_at_best_time = Column("conn_at_best_time", width=21)
                self.columns += [self.time_at_best_conn]
                self.columns += [self.conn_at_best_time]

            if "Longest Subtour" in obj:
                self.best_subtour = Column("best_subtour", width=width)
                self.columns += [self.best_subtour]

            if "Max Disconnected Time" in obj:
                self.best_max_disconn = Column("best_max_disconn", width=16)
                self.columns += [self.best_max_disconn]

            if "Mean Disconnected Time" in obj:
                self.best_mean_disconn = Column("best_mean_disconn", width=17)
                self.columns += [self.best_mean_disconn]

            if "Percentage Disconnectivity" in obj:
                self.best_disconn = Column("best_disconn", width=17)
                self.columns += [self.best_disconn]

            if "TBV" in obj or True:
                self.max_mean_tbv_info = Column("max_mean_tbv_info", width=17)
                self.columns += [self.max_mean_tbv_info]

            if "Path Speed Violations as Objective" in obj:
                self.best_sv = Column("best_sv", width=17)
                self.columns += [self.best_sv]

        self.cv_min = MinimumConstraintViolation()
        self.cv_avg = AverageConstraintViolation()
        self.n_nds = NumberOfNondominatedSolutions()
        self.igd = Column("igd")
        self.gd = Column("gd")
        self.hv = Column("hv")
        self.eps = Column("eps")
        self.indicator = Column("indicator")

        self.pf = None
        self.indicator_no_pf = None

    def update(self, algorithm):
        super().update(algorithm)
        sols = algorithm.pop.get("X")

        if self.best_dist:
            dist_values = [sol[0].total_distance for sol in sols]
            self.best_dist.set(min(dist_values))

        if self.best_time:
            time_values = [sol[0].mission_time for sol in sols]
            self.best_time.set(min(time_values))

        if self.time_at_best_conn:
            conn_values = [sol[0].percentage_connectivity for sol in sols]
            best_conn_idx = conn_values.index(max(conn_values))
            self.time_at_best_conn.set(sols[best_conn_idx][0].mission_time)

        if self.conn_at_best_time:
            time_values = [sol[0].mission_time for sol in sols]
            best_time_idx = time_values.index(min(time_values))
            self.conn_at_best_time.set(sols[best_time_idx][0].percentage_connectivity)

        if self.best_conn:
            conn_values = [sol[0].percentage_connectivity for sol in sols]
            self.best_conn.set(max(conn_values))

        if self.best_max_disconn:
            max_disconn_values = [sol[0].max_disconnected_time for sol in sols]
            self.best_max_disconn.set(min(max_disconn_values))

        if self.best_mean_disconn:
            mean_disconn_values = [sol[0].mean_disconnected_time for sol in sols]
            self.best_mean_disconn.set(min(mean_disconn_values))

        if self.best_disconn:
            mean_disconn_values = [sol[0].percentage_disconnectivity for sol in sols]
            self.best_disconn.set(min(mean_disconn_values))

        if self.max_mean_tbv_info:
            max_mean_tbv_values = [sol[0].max_mean_tbv for sol in sols]
            self.max_mean_tbv_info.set((round(min(max_mean_tbv_values), 1), round(max(max_mean_tbv_values), 1)))

        if self.best_sv:
            sv_values = [sol[0].path_speed_violations for sol in sols]
            self.best_sv.set(min(sv_values))


        if self.best_subtour:
            subtour_values = [sol[0].longest_subtour for sol in sols]
            self.best_subtour.set(min(subtour_values))



        G, H, F = algorithm.pop.get("G", "H", "F")

        G_cvs = np.array(G.tolist())
        H_cvs = np.array(H.tolist())

        cvs = np.hstack((G_cvs, H_cvs))
        sum_cvs = np.sum(cvs, axis=0)

        cvs_df = pd.DataFrame(data=cvs, columns=self.problem.model["G"]+self.problem.model["H"])

        F_df = pd.DataFrame(data=F, columns=self.problem.model["F"])
        
        if len(self.problem.model["G"])!=0 and len(self.problem.model["H"])!=0:
            self.cv_min.set(np.min(sum_cvs))
            self.cv_avg.set(np.mean(sum_cvs))

        super().update(algorithm)

        for col in [self.igd, self.gd, self.hv, self.eps, self.indicator]:
            col.set(None)

        F, feas = algorithm.opt.get("F", "feas")
        F = F[feas]

        if len(F) > 0:

            if self.pf is not None:

                if feas.sum() > 0:
                    self.igd.set(IGD(self.pf, zero_to_one=True).do(F))
                    self.gd.set(GD(self.pf, zero_to_one=True).do(F))

                    if self.hv in self.columns:
                        self.hv.set(Hypervolume(pf=self.pf, zero_to_one=True).do(F))

            if self.indicator_no_pf is not None:

                ind = self.indicator_no_pf
                ind.update(algorithm)

                valid = ind.delta_ideal is not None

                if valid:

                    if ind.delta_ideal > ind.tol:
                        max_from = "ideal"
                        eps = ind.delta_ideal
                    elif ind.delta_nadir > ind.tol:
                        max_from = "nadir"
                        eps = ind.delta_nadir
                    else:
                        max_from = "f"
                        eps = ind.delta_f

                    self.eps.set(eps)
                    self.indicator.set(max_from)


    def initialize(self, algorithm):
        problem = algorithm.problem

        self.columns += [self.n_nds]

        if problem.has_constraints():
            self.columns += [self.cv_min, self.cv_avg]

        self.pf = pareto_front_if_possible(problem)
        if self.pf is not None:
            self.columns += [self.igd, self.gd]

            if problem.n_obj == 2:
                self.columns += [self.hv]

        else:
            self.indicator_no_pf = MultiObjectiveSpaceTermination()
            self.columns += [self.eps, self.indicator]
