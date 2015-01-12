#!/usr/bin/env python

#general libs imports
import json
import string
import wolframalpha
import mysql.connector as sql
class AIbot:
	def __init__(self,app_id):
		self.client = wolframalpha.Client(app_id)
	
	def answer_wolframalpha(self,question):
		res = self.client.query(question)
		if len(res.pods) > 0:
			texts = ""
			pod = res.pods[1]
			if pod.text:
				texts = pod.text
			else:
				texts = "I have no answer for that"
			# to skip ascii character in case of error
			texts = texts.encode('ascii', 'ignore')
		else:
			texts= "Sorry, I am not sure."
		return texts
	
	def answer_local_db(self,question):
		#remove redundant spaces
		keywords=question.strip().rstrip().split()
		key=' %'
		key='\''+key.join(keywords)+'%\''
		#connect to mysql and execute a "SELECT" query	
		connection=sql.connect(user='root',password='root',database='python_db')
		cursor = connection.cursor()
		query = ("SELECT ans FROM questions ""WHERE ques LIKE "+key)
		cursor.execute(query)
		results=list(cursor)
		cursor.close()
		connection.close()
		#reformatting
		for i in range(0,len(results)):
			results[i]=str(results[i]).strip('(u\'').rstrip('\',)')
		#in case of multiple matches, use rand
		if "END_ENTERACTION" in results:
			return "END_ENTERACTION"
		else:
			return results
