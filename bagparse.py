from bagpy import bagreader
import pandas as pd

b = bagreader("2022-10-31-08-29-01.bag")
print(b.topic_table)

chatter_csv = b.message_by_topic("/chatter")
df = pd.read_csv(chatter_csv)
print(df)