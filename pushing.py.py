# -*- coding: utf-8 -*-
"""
Created on Tue Feb 19 11:37:08 2019

@author: ATI-2 Sramana Dan

"""

from git import Repo



repo_dir = 'grip'
repo = Repo(repo_dir)
file_list = ['image0.jpg']
commit_message = 'Adding catheter image'
repo.index.add(file_list)
repo.git.add(update=True)
repo.index.commit(commit_message)
origin = repo.remote('origin')
origin.push()

