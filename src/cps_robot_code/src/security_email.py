	'''
	@authors: Matthew Pryke
	'''

from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart
from email.mime.base import MIMEBase
from email import encoders
import smtplib
import os


class Security_Email():
	def __init__(self):
		self.email_sender = "roomsecure121@gmail.com"
		self.email_reciever = ["Tkdmattpryke@outlook.com"]
	

	def create_server(self):
	'''
	Creates the smpt server that the email can send from
	:return: email server
	'''
		email_sender_password = "poeg nxea ultc eagm"
		smtp_port = 587
		smtp_server = "smtp.gmail.com"

		server = smtplib.SMTP(smtp_server, smtp_port)


		server.starttls()
		server.login(self.email_sender, email_sender_password)

		return server


	def email_structure(self, person):
	'''
	creates the structure of the email including the body and attachments.
	:param person: recieving email
	:return: email structure
	'''
		subject = "Security Alert"
		body = f"""
		Hi Bossman,

		There is someome in the secure room.
		Attached is a pic of the person inside.
		BTW the room isnt secure if someone is in it.

		All the best,
		the best secutiy company
		"""

		message = MIMEMultipart()
		message["From"] = self.email_sender
		message["To"] = person
		message["Subject"] = subject
		message.attach(MIMEText(body, "plain"))

		attachment = MIMEBase("application", "octet-stream")
		with open("security_threat.jpg", "rb") as image:
			attachment.set_payload(image.read())
		encoders.encode_base64(attachment)
		attachment.add_header("Content-Disposition", 
			"attachment; filename=security_threat.jpg")
		message.attach(attachment)

		message = message.as_string()

		return message


	def send_email(self):
	'''
	sends the email to the recievers
	'''
		server = self.create_server()
		for person in self.email_reciever:
			message = self.email_structure(person)
			server.sendmail(self.email_sender, self.email_reciever, message)
		self.clear_up(server)


	def clear_up(self, server):
	'''
	ends the server as it is no longer needed.
	deletes the images as it is no longer needed.
	'''
		server.quit()
		# os.remove("security_threat.jpg")




if __name__ == "__main__":
	Security_Email().send_email()
