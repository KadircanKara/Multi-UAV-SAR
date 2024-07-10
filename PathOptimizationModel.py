# from PathSolution import *
# from Distance import *
# from Connectivity import *
# from Time import *

moo_model_without_disconn = {
    'Type': 'MOO',
    'Exp':'dist_conn_disconn',
    'Alg': "NSGA2",
    'F': ['Total Distance','Percentage Connectivity'],
    'G': ['Limit Long Jumps','Limit Max Longest Subtour'], # 'Limit Subtour Range' 'Limit Long Jumps'
    'H': ['Enforce Hovering Connectivity'] # 'No Long Jumps'
}

moo_model_with_disconn = {
    'Type': 'MOO',
    'Exp':'dist_conn',
    'Alg': "NSGA2",
    'F': ['Total Distance','Percentage Connectivity','Mean Disconnected Time', 'Max Disconnected Time'],
    'G': ['Limit Long Jumps'], # 'Limit Subtour Range' 'Limit Long Jumps' 'Limit Max Longest Subtour'
    'H': ['Enforce Hovering Connectivity'] # Full connectivity of hovering nodes
}

moo_model_mtsp = {
    'Type': 'MOO',
    'Exp': 'dist_subtour',
    'Alg': "NSGA2",
    'F': ['Total Distance', 'Longest Subtour'],
    'G': ['Limit Long Jumps'],
    'H': []  # 'No Long Jumps', 'No Extra Revisits'

}

distance_soo_model = {
    'Type': 'SOO',
    'Exp': 'dist',
    'Alg': "GA",
    'F': ['Total Distance'],
    'G': ['Limit Long Jumps'],
    'H': []  # 'No Long Jumps', 'No Extra Revisits', 'Number of Visits Hard Constraint'
}
