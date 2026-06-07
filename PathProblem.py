import autograd.numpy as anp
import types

from PathOptimizationModel import *
from PathSolution import *
from PathInfo import *
from Distance import *
from Connectivity import *
from PathFuncDict import *
from PathRepair import *

from pymoo.core.problem import ElementwiseProblem

class AdaptiveRelayProblem(ElementwiseProblem):

    def __init__(self, info, elementwise=True, **kwargs):
        self.model = info.model # My addition
        self.info = info
        self.n_var = self.info.number_of_drones - 1
        self.xl = 0
        self.xu = self.info.number_of_cells - 1
        self.n_obj = len(self.model['F'])
        self.n_ieq_constr = len(self.model['G'])
        self.n_eq_constr = len(self.model['H'])

        super().__init__(n_var = self.n_var, n_obj=self.n_obj, n_ieq_constr=self.n_ieq_constr, n_eq_constr=self.n_eq_constr, elementwise=True, **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):

        sol = x
        model = self.model
        f,g,h=[],[],[]

        if len(model["F"])==1 and '-' in model["F"][0]:
            f = [0]
        else:
            for i in range(self.n_obj):
                obj_name = self.model['F'][i]
                obj_calc = model_metric_info["Objectives"][obj_name]
                if isinstance(obj_calc, types.FunctionType):
                    f.append(obj_calc(sol))
                else:
                    f.append(obj_calc)

        for j in range(self.n_ieq_constr):
            ieq_constr_name = self.model['G'][j]
            ieq_constr_calc = model_metric_info["Constraints"][ieq_constr_name]
            if isinstance(ieq_constr_calc, types.FunctionType):
                g.append(ieq_constr_calc(sol))
            else:
                g.append(ieq_constr_calc)

        for k in range(self.n_eq_constr):
            eq_constr_name = self.model['H'][k]
            eq_constr_calc = model_metric_info["Constraints"][eq_constr_name]
            if isinstance(eq_constr_calc, types.FunctionType):
                h.append(eq_constr_calc(sol))
            else:
                h.append(eq_constr_calc)

        if f:
            out['F'] = anp.column_stack(f)
        if g:
            out['G'] = anp.column_stack(g)
        if h:
            out['H'] = anp.column_stack(h)


class PathProblem(ElementwiseProblem):

    def __init__(self, info:PathInfo, elementwise=True, **kwargs):
        self.model = info.model # My addition
        self.info = info
        self.n_var = 1
        self.n_obj = len(self.model['F'])
        self.n_ieq_constr = len(self.model['G'])
        self.n_eq_constr = len(self.model['H'])


        super().__init__(n_var = self.n_var, n_obj=self.n_obj, n_ieq_constr=self.n_ieq_constr, n_eq_constr=self.n_eq_constr, elementwise=True, **kwargs)

    def _evaluate(self, x, out, *args, **kwargs):

        sol:PathSolution = x[0]
        model = self.model
        f,g,h=[],[],[]

        if model['Type']=="WS":
            objectives = get_objectives_from_weighted_sum_model(model)
            score = 0
            for i in range(len(objectives)):
                obj_name = objectives[i]
                obj_calc = model_metric_info["Objectives"][obj_name][0]
                obj_pol = model_metric_info["Objectives"][obj_name][1]
                score += obj_calc(sol)/get_objective_normalization_factors(sol.info)[obj_name]*obj_pol
            f.append(score)

        else:
            for i in range(self.n_obj):
                obj_name = self.model['F'][i]
                obj_calc = model_metric_info["Objectives"][obj_name][0]
                obj_pol = model_metric_info["Objectives"][obj_name][1]
                f.append(obj_calc(sol)*obj_pol)

        for j in range(self.n_ieq_constr):
            ieq_constr_name = self.model['G'][j]
            ieq_constr_calc = model_metric_info["Constraints"][ieq_constr_name]
            g.append(ieq_constr_calc(sol))

        for k in range(self.n_eq_constr):
            eq_constr_name = self.model['H'][k]
            eq_constr_calc = model_metric_info["Constraints"][eq_constr_name]
            h.append(eq_constr_calc(sol))

        if f:
            out['F'] = anp.column_stack(f)
        if g:
            out['G'] = anp.column_stack(g)
        if h:
            out['H'] = anp.column_stack(h)