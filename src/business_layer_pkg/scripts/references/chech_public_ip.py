
import requests
    
    
def get_public_ip():
   try:
      response = requests.get('https://api.ipify.org?format=json')
      response.raise_for_status()  # Raise an exception for HTTP errors
      ip_info = response.json()
      return ip_info['ip']
   except requests.RequestException as e:
      print(f"Error occurred: {e}")
      return None
   
if __name__ == "__main__":
   print(get_public_ip())