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

        objs = problem.model["F"]

        # for obj in objs:
        #     self.model_metric_info[obj][1] = None

        self.best_dist = None
        self.best_conn = None
        self.best_mean_disconn = None
        self.best_max_disconn = None

        # self.min_dist = None
        # self.max_dist = None
        # self.mean_dist = None
        # # self.min_distPenalty = None
        # # self.min_distPenalty = None
        # # self.max_distPenalty = None
        # self.min_perc_conn = None
        # self.max_perc_conn = None
        # self.mean_perc_conn = None
        # self.min_maxDisconnectedTime = None
        # self.max_maxDisconnectedTime = None
        # self.mean_maxDisconnectedTime = None
        # self.min_meanDisconnectedTime = None
        # self.max_meanDisconnectedTime = None
        # self.mean_meanDisconnectedTime = None
        #
        #
        width = 13

        if "Total Distance" in objs:
            # print("Distance Output In !")
            self.best_dist = Column("best_dist", width=width)
            # self.min_dist = Column("min_dist", width=13)
            # self.max_dist = Column("max_dist", width=13)
            # self.mean_dist = Column("mean_dist", width=13)
            # self.min_dist = Column("min_dist", width=len("min_dist"))
            # self.max_dist = Column("max_dist", width=len("max_dist"))
            # self.mean_dist = Column("mean_dist", width=len("mean_dist"))
            self.columns += [self.best_dist]
        #
        if "Percentage Connectivity" in objs:
            # print("Connectivity Output In !")
            self.best_conn = Column("best_conn", width=width)
            # self.min_perc_conn = Column("min_perc_conn", width=13)
            # self.max_perc_conn = Column("max_perc_conn", width=13)
            # self.mean_perc_conn = Column("mean_perc_conn", width=15)
            # self.min_perc_conn = Column("min_perc_conn", width=len("min_perc_conn"))
            # self.max_perc_conn = Column("max_perc_conn", width=len("max_perc_conn"))
            # self.mean_perc_conn = Column("mean_perc_conn", width=len("mean_perc_conn"))
            self.columns += [self.best_conn]
        #
        if "Max Disconnected Time" in objs:
            self.best_max_disconn = Column("best_max_disconn", width=16)
            # self.min_maxDisconnectedTime = Column("min_maxDisconnTime", width=17)
            # self.max_maxDisconnectedTime = Column("max_maxDisconnTime", width=17)
            # self.mean_maxDisconnectedTime = Column("mean_maxDisconnTime", width=17)
            # self.min_disconnected_time = Column("min_disconn_time", width=len("min_disconn_time"))
            # self.max_disconnected_time = Column("max_disconn_time", width=len("max_disconn_time"))
            # self.mean_disconnected_time = Column("mean_disconn_time", width=len("mean_disconn_time"))
            self.columns += [self.best_max_disconn]
        #
        if "Mean Disconnected Time" in objs:
            self.best_mean_disconn = Column("best_mean_disconn", width=17)
            # self.min_meanDisconnectedTime = Column("min_meanDisconnTime", width=17)
            # self.max_meanDisconnectedTime = Column("max_meanDisconnTime", width=17)
            # self.mean_meanDisconnectedTime = Column("mean_meanDisconnTime", width=17)
            # self.min_disconnected_time = Column("min_disconn_time", width=len("min_disconn_time"))
            # self.max_disconnected_time = Column("max_disconn_time", width=len("max_disconn_time"))
            # self.mean_disconnected_time = Column("mean_disconn_time", width=len("mean_disconn_time"))
            self.columns += [self.best_mean_disconn]


        # FROM MULTI

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

        # self.columns += [self.cv_min, self.cv_avg]

    def update(self, algorithm):
        super().update(algorithm)
        sols = algorithm.pop.get("X")
        # print(f"sols: {sols}")

        # sol = PathSolution()
        # sol.


        if self.best_dist:
            dist_values = [sol[0].total_distance for sol in sols]
            self.best_dist.set(min(dist_values))
            # self.max_dist.set(max(dist_values))
            # self.mean_dist.set(np.mean(dist_values))

        if self.best_conn:

            conn_values = [sol[0].percentage_connectivity for sol in sols]
            # print(f"perc conn values: {conn_values}")
            self.best_conn.set(max(conn_values))
            # self.max_perc_conn.set(max(perc_conn_values))
            # self.mean_perc_conn.set(np.mean(perc_conn_values))

        if self.best_max_disconn:
            max_disconn_values = [sol[0].max_disconnected_time for sol in sols]
            self.best_max_disconn.set(min(max_disconn_values))
            # self.max_maxDisconnectedTime.set(max(max_disconnected_time_values))
            # self.mean_maxDisconnectedTime.set(np.mean(max_disconnected_time_values))

        if self.best_mean_disconn:
            mean_disconn_values = [sol[0].mean_disconnected_time for sol in sols]
            self.best_mean_disconn.set(min(mean_disconn_values))
            # self.max_maxDisconnectedTime.set(max(mean_disconn_values))
            # self.mean_maxDisconnectedTime.set(np.mean(mean_disconn_values))


        G, H = algorithm.pop.get("G", "H")

        cvs = G.tolist()

        self.cv_min.set(int(min(cvs)[0]))
        self.cv_avg.set(np.mean(cvs))

        # FROM MULTI

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


    # FROM MULTI

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
